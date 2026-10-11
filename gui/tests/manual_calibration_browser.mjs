// 离线浏览器验证：使用已有 Playwright 与浏览器，不安装依赖、不连接硬件。
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import {clientToImage} from '../manual_calibration.js';

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
      assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=document.documentElement.clientWidth),'page has horizontal overflow');
      await page.screenshot({path:path.join(outputDir,scenario.name+'.png'),fullPage:true});
      // 重新上传必须清除点、H、预览和保存按钮。
      await page.locator('#upload').setInputFiles(imagePath);await page.waitForFunction(()=>document.querySelectorAll('#points li').length===0);
      assert.equal(await page.locator('#save').isDisabled(),true);assert.equal(await page.locator('#preview-panel').isHidden(),true);
      console.log('PASS: '+scenario.name+' actual browser clicks, native pixels, invalidation and preview.');
      await context.close();
    }
    assert.deepEqual(failures,[],'browser JS errors');
  } finally {await browser.close();}
}
