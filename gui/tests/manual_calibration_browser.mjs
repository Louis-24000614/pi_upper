// 离线浏览器验证：使用已有 Playwright 与浏览器，不安装依赖、不连接硬件。
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import {clientToImage, compareCheckPoint} from '../manual_calibration.js';

// 独立的已知投影与实测值，避免用网页的计算反推期望值。
const baseline={image_size:[1280,720],H_img_to_vehicle_ground_m:[[.01,0,-.5],[0,-.01,1],[0,0,1]],
  H_img_to_navigation_bev_px:[[99,0,0],[0,99,0],[0,0,1]]};
const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-10,`${a} != ${b}`);
let comparison=compareCheckPoint(baseline,[1280,720],[80,40],[.32,.58],[.31,.57]);
comparison.oldGround.forEach((v,i)=>near(v,[.3,.6][i]));
comparison.oldError.xy.forEach((v,i)=>near(v,[-.01,.03][i]));
comparison.newError.xy.forEach((v,i)=>near(v,[.01,.01][i]));
near(comparison.oldError.distance,Math.hypot(.01,.03));near(comparison.newError.distance,Math.hypot(.01,.01));
assert.equal(comparison.oldUnavailable,null);
const perspective={...baseline,H_img_to_vehicle_ground_m:[[2,0,1],[0,3,2],[.01,.02,1]]};
comparison=compareCheckPoint(perspective,[1280,720],[10,20],null);
near(comparison.oldGround[0],21/1.5);near(comparison.oldGround[1],62/1.5);
assert.equal(comparison.newError,null);assert.equal(comparison.oldError,null);
const equivalent={...perspective,H_img_to_vehicle_ground_m:perspective.H_img_to_vehicle_ground_m.map(row=>row.map(v=>-7*v))};
assert.deepEqual(compareCheckPoint(equivalent,[1280,720],[10,20],null).oldGround,comparison.oldGround);
for(const invalid of [
  {...baseline,image_size:[640,360]}, {...baseline,error:'旧参数无效'},
  {...baseline,H_img_to_vehicle_ground_m:[[1,2]]},
  {...baseline,H_img_to_vehicle_ground_m:[[NaN,0,1],[0,1,0],[0,0,1]]},
  {...baseline,H_img_to_vehicle_ground_m:[[1,0,0],[0,1,0],[0,0,0]]},
  {...baseline,H_img_to_vehicle_ground_m:[[1,0,0],[0,1,0],[0,0,1e-12]]},
]) {
  const result=compareCheckPoint(invalid,[1280,720],[80,40],[.32,.58],[.31,.57]);
  assert.equal(result.oldGround,null);assert.equal(result.oldError,null);assert.ok(result.oldUnavailable);
  near(result.newError.distance,Math.hypot(.01,.01));
}
assert.equal(compareCheckPoint(baseline,[640,360],[80,40],null).oldGround,null);
console.log('PASS: old/new ground projection, homogeneous division, signed errors and unavailable baseline.');

// 独立校验 CSS 缩放、留白、平移、放大和移动屏幕的逆变换。
for (const [logical, rect, view] of [
  [[900,500], {left:40,top:80,width:900,height:500}, {x:5.55,y:0,scale:500/720}],
  [[900,500], {left:40,top:80,width:675,height:375}, {x:-140,y:-90,scale:1.25}],
  [[360,300], {left:12,top:90,width:360,height:300}, {x:0,y:48.75,scale:360/1280}],
]) {
  const original=[370.25,280.75];
  const client=[rect.left+(view.x+original[0]*view.scale)*rect.width/logical[0],rect.top+(view.y+original[1]*view.scale)*rect.height/logical[1]];
  const actual=clientToImage(...client,rect,logical,view);
  assert.ok(actual.every((v,i)=>Math.abs(v-original[i])<1e-9));
}
if (process.argv.length === 2) {
  console.log('PASS: display transform (CSS zoom / letterboxing / zoom / pan / mobile).');
} else {
  const [url,imagePath,outputDir,runtimeModule]=process.argv.slice(2);
  assert.ok(url&&imagePath&&outputDir&&runtimeModule,'Arguments: URL synthetic.png output-directory existing-playwright/index.mjs');
  const {chromium}=await import(pathToFileURL(path.resolve(runtimeModule)).href);
  const browser=await chromium.launch({channel:process.env.MANUAL_BROWSER_CHANNEL||'msedge',headless:true});
  const failures=[];
  await fs.mkdir(outputDir,{recursive:true});
  const corners=[[270,150],[900,180],[1080,590],[160,560]];
  try {
    for (const scenario of [
      {name:'desktop',viewport:{width:1360,height:980},dpr:1,cssZoom:1},
      {name:'zoom75-pan50',viewport:{width:1360,height:980},dpr:1.25,cssZoom:.75,pan:true,zoom:50},
      {name:'mobile',viewport:{width:390,height:844},dpr:2,cssZoom:1,mobile:true},
    ]) {
      const context=await browser.newContext({viewport:scenario.viewport,deviceScaleFactor:scenario.dpr,isMobile:!!scenario.mobile,hasTouch:!!scenario.mobile});
      const page=await context.newPage();page.on('pageerror',e=>failures.push(e.message));
      await page.goto(url);await page.waitForFunction(()=>document.querySelector('#old-ground').textContent.includes('e'));
      assert.equal(await page.locator('#capture').isDisabled(),true,'offline must not open a camera');
      await page.locator('#upload').setInputFiles(imagePath);await page.locator('#viewport').waitFor({state:'visible'});
      await page.evaluate(z=>{document.body.style.zoom=z;},scenario.cssZoom);
      if(scenario.zoom)await page.locator('#zoom').evaluate((el,value)=>{el.value=value;el.dispatchEvent(new Event('input',{bubbles:true}));},scenario.zoom);
      await page.waitForTimeout(150);
      let geometry=await page.locator('#canvas').evaluate(el=>{
        const r=el.getBoundingClientRect();return {rect:{left:r.left,top:r.top,width:r.width,height:r.height},width:el.clientWidth,height:el.clientHeight};
      });
      let scale=Math.min(geometry.width/1280,geometry.height/720)*(scenario.zoom||100)/100;
      let offset=[(geometry.width-1280*scale)/2,(geometry.height-720*scale)/2];
      function location(p){return {x:geometry.rect.left+(offset[0]+p[0]*scale)*geometry.rect.width/geometry.width,y:geometry.rect.top+(offset[1]+p[1]*scale)*geometry.rect.height/geometry.height};}
      // 在留白处点击不能污染原图角点。
      const blank=offset[0]>5?{x:geometry.rect.left+2,y:geometry.rect.top+geometry.rect.height/2}:{x:geometry.rect.left+geometry.rect.width/2,y:geometry.rect.top+2};
      await page.mouse.click(blank.x,blank.y);assert.equal(await page.locator('#points li').count(),0);
      if(scenario.pan){
        await page.locator('#pan').click();
        const start={x:geometry.rect.left+geometry.rect.width/2,y:geometry.rect.top+geometry.rect.height/2};
        await page.mouse.move(start.x,start.y);await page.mouse.down();await page.mouse.move(start.x+24,start.y+15,{steps:4});await page.mouse.up();
        offset[0]+=24*geometry.width/geometry.rect.width;offset[1]+=15*geometry.height/geometry.rect.height;
        await page.locator('#pan').click();
      }
      for(const p of corners){const at=location(p);if(scenario.mobile)await page.touchscreen.tap(at.x,at.y);else await page.mouse.click(at.x,at.y);}
      await page.waitForFunction(()=>document.querySelector('#save').disabled===false);
      const measured=await page.locator('#points li').evaluateAll(els=>els.map(el=>[Number(el.dataset.x),Number(el.dataset.y)]));
      assert.equal(measured.length,4);
      const tolerance=Math.max(1.5,1.5/scale);
      assert.ok(measured.every((p,i)=>p.every((v,j)=>Math.abs(v-corners[i][j])<tolerance)),scenario.name+' original pixel mismatch');
      let record=JSON.parse(await page.locator('#record').textContent());assert.equal(record.actual_width_m,.09);assert.equal(record.actual_height_m,.06);
      if(scenario.name==='desktop'){
        await page.locator('#zoom').evaluate(el=>{el.value=160;el.dispatchEvent(new Event('input',{bubbles:true}));});
        const afterZoom=await page.locator('#points li').evaluateAll(els=>els.map(el=>[Number(el.dataset.x),Number(el.dataset.y)]));
        assert.deepEqual(afterZoom,measured,'zoom must preserve previously selected original pixels');
        scale=Math.min(geometry.width/1280,geometry.height/720)*1.6;
        offset=[(geometry.width-1280*scale)/2,(geometry.height-720*scale)/2];
        // 拖动角点，旧 H 立即失效；修正坐标仍为原图像素。
        const start=location(corners[0]),end=location([280,155]);
        await page.mouse.move(start.x,start.y);await page.mouse.down();await page.mouse.move(end.x,end.y,{steps:4});await page.mouse.up();
        await page.waitForFunction(()=>!document.querySelector('#save').disabled);
        const moved=await page.locator('#points li').first().evaluate(el=>[Number(el.dataset.x),Number(el.dataset.y)]);
        assert.ok(Math.abs(moved[0]-280)<tolerance&&Math.abs(moved[1]-155)<tolerance);
        await page.locator('#fit').click();
        await page.locator('#columns').fill('0');await page.waitForFunction(()=>document.querySelector('#error').textContent.includes('正整数'));
        assert.equal(await page.locator('#save').isDisabled(),true);
        await page.locator('#columns').fill('3');await page.waitForFunction(()=>!document.querySelector('#save').disabled);
        await page.locator('summary').filter({hasText:'可选：另存车辆坐标候选'}).click();
        await page.locator('#aligned').check();await page.locator('#center-x').fill('3');await page.locator('#center-y').fill('40');
        await page.waitForFunction(()=>!document.querySelector('#candidate-panel').hidden&&!document.querySelector('#save').disabled);
        await page.locator('#save').click();await page.waitForFunction(()=>document.querySelector('#status').textContent.startsWith('已独立保存：'));
        const file=(await page.locator('#status').textContent()).slice('已独立保存：'.length);
        const saved=JSON.parse(await fs.readFile(file,'utf8'));
        assert.equal(saved.coordinate_reference,'selected_region_center');assert.equal(saved.vehicle_candidate_file,'vehicle_candidate.json');
        const candidate=JSON.parse(await fs.readFile(path.join(path.dirname(file),'vehicle_candidate.json'),'utf8'));
        assert.deepEqual(candidate.region_center_m,[.03,.4]);assert.equal(candidate.verified,false);
      }
      if(await page.locator('#candidate-panel').isHidden()){
        await page.locator('summary').filter({hasText:'可选：另存车辆坐标候选'}).click();
        await page.locator('#aligned').check();await page.locator('#center-x').fill('3');await page.locator('#center-y').fill('40');
        await page.waitForFunction(()=>!document.querySelector('#candidate-panel').hidden&&!document.querySelector('#save').disabled);
      }
      await page.locator('#check-mode').click();await page.locator('#canvas').scrollIntoViewIfNeeded();
      geometry=await page.locator('#canvas').evaluate(el=>{const r=el.getBoundingClientRect();return {rect:{left:r.left,top:r.top,width:r.width,height:r.height},width:el.clientWidth,height:el.clientHeight};});
      // 原棋盘区域的对角线交点为独立中心检查点，未参加四角拟合。
      const a=corners[0],b=corners[2],c=corners[1],d=corners[3];
      const cross=(a,b)=>a[0]*b[1]-a[1]*b[0],ab=[b[0]-a[0],b[1]-a[1]],cd=[d[0]-c[0],d[1]-c[1]];
      const t=cross([c[0]-a[0],c[1]-a[1]],cd)/cross(ab,cd),center=[a[0]+t*ab[0],a[1]+t*ab[1]];
      if(scenario.name==='desktop'){scale=Math.min(geometry.width/1280,geometry.height/720);offset=[(geometry.width-1280*scale)/2,(geometry.height-720*scale)/2];}
      const at=location(center);if(scenario.mobile)await page.touchscreen.tap(at.x,at.y);else await page.mouse.click(at.x,at.y);
      await page.waitForFunction(()=>document.querySelector('#check-coordinate').textContent.includes('车辆 (X,Y)'));
      assert.equal(await page.locator('#points li').count(),4,'check must not change fitting corners');
      assert.ok((await page.locator('#check-comparison').textContent()).includes('旧 H · 参数估算'));
      assert.ok((await page.locator('#check-comparison').textContent()).includes('新 H · 车辆候选'));
      const measurementResponse=page.waitForResponse(response=>response.url().endsWith('/check-point')&&response.request().method()==='POST');
      await page.locator('#check-x').fill('4');await page.locator('#check-y').fill('38');await page.locator('#record-check').click();
      const measurement=await (await measurementResponse).json();
      await page.waitForFunction(()=>document.querySelectorAll('#checks li').length===1);
      assert.ok((await page.locator('#checks').textContent()).includes('总误差'));
      const oldH=record.baseline.H_img_to_vehicle_ground_m,pixel=measurement.image_point;
      const homogeneous=oldH.map(row=>row[0]*pixel[0]+row[1]*pixel[1]+row[2]);
      const oldGround=homogeneous.slice(0,2).map(v=>v/homogeneous[2]),actual=[.04,.38];
      const errorCells=predicted=>{const error=predicted.map((v,i)=>v-actual[i]);return [...predicted,...error,Math.hypot(...error)].map(v=>(v*100).toFixed(2));};
      const expectedCells=[['4.00','38.00','—','—','—'],errorCells(oldGround),errorCells(measurement.vehicle_ground_m)];
      const tableCells=selector=>page.locator(selector).evaluateAll(rows=>rows.map(row=>Array.from(row.querySelectorAll('td')).map(cell=>cell.textContent)));
      assert.deepEqual(await tableCells('#check-comparison tbody tr'),expectedCells,'current comparison must use the same pixel and measured point');
      assert.deepEqual(await tableCells('#checks tbody tr'),expectedCells,'recorded comparison must match current point');
      assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=document.documentElement.clientWidth),'comparison must not overflow the page');
      await page.locator('#check-comparison').screenshot({path:path.join(outputDir,scenario.name+'-comparison.png')});
      if(scenario.mobile){
        const scroll=page.locator('#check-comparison .comparison-scroll');
        assert.ok(await scroll.evaluate(el=>el.scrollWidth>el.clientWidth),'mobile comparison must scroll within its table');
        await scroll.evaluate(el=>{el.scrollLeft=el.scrollWidth;});
        await page.locator('#check-comparison').screenshot({path:path.join(outputDir,scenario.name+'-comparison-errors.png')});
      }
      // 修改实测输入不能留下上一份误差；已记录的检查仍保留原始实测值。
      await page.locator('#check-x').fill('5');
      const edited=await tableCells('#check-comparison tbody tr');
      assert.deepEqual(edited[0],['—','—','—','—','—']);assert.deepEqual(edited[1].slice(2),['—','—','—']);
      assert.deepEqual(await tableCells('#checks tbody tr'),expectedCells);
      await page.locator('#check-x').fill('4');
      await page.locator('#check-mode').click();await page.locator('#check-mode').click();
      await page.locator('#canvas').scrollIntoViewIfNeeded();
      const pointBox=await page.locator('#canvas').boundingBox();
      const another={x:pointBox.x+(offset[0]+(center[0]+20)*scale)*geometry.rect.width/geometry.width,
        y:pointBox.y+(offset[1]+center[1]*scale)*geometry.rect.height/geometry.height};
      const nextResponse=page.waitForResponse(response=>response.url().endsWith('/check-point'));
      if(scenario.mobile)await page.touchscreen.tap(another.x,another.y);else await page.mouse.click(another.x,another.y);
      await nextResponse;assert.equal(await page.locator('#check-x').inputValue(),'');assert.equal(await page.locator('#check-y').inputValue(),'');
      assert.deepEqual(await tableCells('#check-comparison tbody tr').then(rows=>rows[0]),['—','—','—','—','—']);
      assert.equal(await page.locator('#apply').isDisabled(),true,'review confirmations must be explicit');
      await page.locator('#measurement-reviewed').check();await page.locator('#image-source-confirmed').check();await page.locator('#ground-contact').check();
      const status=await (await page.request.get(url+'/status')).json();
      if(status.can_apply){
        await page.locator('#apply').click();await page.waitForFunction(()=>document.querySelector('#status').textContent.startsWith('已选择标定'));
        const selected=await (await page.request.get(url+'/status')).json();
        assert.equal(selected.active_calibration.candidate.coordinate_reference,'navigation_camera_ground_projection');
        assert.equal(selected.active_calibration.checks.length,1);assert.equal(selected.active_calibration.ground_contact_verified,true);
        await page.locator('#restore').click();await page.waitForFunction(()=>document.querySelector('#status').textContent.startsWith('已恢复'));
        assert.equal((await (await page.request.get(url+'/status')).json()).active_calibration,null);
        console.log('PASS: '+scenario.name+' measurement / explicit application / restoration.');
      }else assert.equal(await page.locator('#apply').isDisabled(),true,'custom output cannot select production navigation');
      // 重算也必须清除检查记录和旧对比结果。
      await page.locator('#compute').click();await page.waitForFunction(()=>!document.querySelector('#save').disabled);
      assert.equal(await page.locator('#check-comparison table').count(),0);
      assert.equal(await page.locator('#checks li').count(),0);
      await page.locator('#cell-size').fill('31');await page.waitForFunction(()=>document.querySelectorAll('#checks li').length===0);
      assert.equal(await page.locator('#check-comparison table').count(),0);
      assert.equal(await page.locator('#apply').isDisabled(),true,'input change must clear checks and application eligibility');
      assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=document.documentElement.clientWidth),'page has horizontal overflow');
      await page.screenshot({path:path.join(outputDir,scenario.name+'.png'),fullPage:true});
      // 重新上传必须清除点、H、预览和保存按钮。
      await page.locator('#upload').setInputFiles(imagePath);await page.waitForFunction(()=>document.querySelectorAll('#points li').length===0);
      assert.equal(await page.locator('#save').isDisabled(),true);assert.equal(await page.locator('#preview-panel').isHidden(),true);
      assert.equal(await page.locator('#check-comparison table').count(),0);
      console.log('PASS: '+scenario.name+' actual browser clicks, native pixels, invalidation and preview.');
      await context.close();
    }
    // 分辨率不同的离线上传仍可检查新 H，旧 H 须显示不可用。
    const context=await browser.newContext({viewport:{width:1360,height:980}}),page=await context.newPage();
    await page.goto(url);await page.locator('#upload').setInputFiles(imagePath);await page.locator('#viewport').waitFor({state:'visible'});
    const smallPng=await page.evaluate(async()=>{
      const state=await (await fetch('/status')).json(),image=new Image();image.src='/frozen.png?id='+state.frame_id;await image.decode();
      const canvas=document.createElement('canvas');canvas.width=640;canvas.height=360;
      canvas.getContext('2d').drawImage(image,0,0,640,360);return canvas.toDataURL('image/png').split(',')[1];
    });
    await page.locator('#upload').setInputFiles({name:'small.png',mimeType:'image/png',buffer:Buffer.from(smallPng,'base64')});
    await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('640 × 360'));
    await page.locator('summary').filter({hasText:'可选：另存车辆坐标候选'}).click();
    await page.locator('#aligned').check();await page.locator('#center-x').fill('3');await page.locator('#center-y').fill('40');
    const canvas=page.locator('#canvas'),box=await canvas.boundingBox();
    const dimensions=await canvas.evaluate(el=>[el.clientWidth,el.clientHeight]),scale=Math.min(dimensions[0]/640,dimensions[1]/360);
    const offset=[(dimensions[0]-640*scale)/2,(dimensions[1]-360*scale)/2];
    for(const p of corners)await page.mouse.click(box.x+offset[0]+p[0]/2*scale,box.y+offset[1]+p[1]/2*scale);
    await page.waitForFunction(()=>!document.querySelector('#save').disabled);await page.locator('#check-mode').click();
    await canvas.scrollIntoViewIfNeeded();const checkBox=await canvas.boundingBox();
    await page.mouse.click(checkBox.x+offset[0]+300*scale,checkBox.y+offset[1]+200*scale);
    await page.waitForFunction(()=>document.querySelector('#check-comparison').textContent.includes('分辨率不同'));
    assert.equal(await page.locator('#check-comparison tbody tr').nth(1).locator('td').first().textContent(),'—');
    assert.notEqual(await page.locator('#check-comparison tbody tr').nth(2).locator('td').first().textContent(),'—');
    await context.close();console.log('PASS: browser comparison explicitly disables incompatible baseline.');
    assert.deepEqual(failures,[],'browser JS errors');
  } finally {await browser.close();}
}
