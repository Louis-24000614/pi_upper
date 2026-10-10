// Canvas 使用原始像素点，显示变换与标定数据分离；不对角点自动排序。
export function clientToImage(clientX, clientY, rect, logicalSize, view) {
  return [(clientX - rect.left) * logicalSize[0] / rect.width - view.x,
          (clientY - rect.top) * logicalSize[1] / rect.height - view.y].map(v => v / view.scale);
}

function initPage() {
  const $ = id => document.getElementById(id);
  const canvas = $('canvas'), context = canvas.getContext('2d');
  const names = ['左上', '右上', '右下', '左下'];
  let frameId = null, resultId = null, revision = 0, points = [], image = null;
  let busy = false, saved = false, panMode = false, drag = null, autoTimer = null, invalidateTimer = null;
  let canCapture = false, offline = true, baseline = null;
  let checkMode = false, checkPoint = null, checks = [], hasVehicle = false, active = null, canApply = false, restoreToken = null;
  let view = {x: 0, y: 0, scale: 1}, fitScale = 1;

  async function post(path, body) {
    const response = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const data = await response.json();
    if (!response.ok) throw Error(data.message || '请求失败');
    return data;
  }
  function matrix(value) { return value ? value.map(row => row.map(v => Number(v).toExponential(9)).join('   ')).join('\n') : '无法获得当前矩阵'; }
  function controls() {
    $('capture').disabled = busy || !canCapture;
    $('upload').disabled = busy;
    $('compute').disabled = busy || !frameId || points.length !== 4;
    $('save').disabled = busy || !resultId || saved;
    $('check-mode').disabled = busy || !resultId;
    $('record-check').disabled = busy || !resultId || !checkPoint || !hasVehicle;
    $('apply').disabled = busy || !canApply || !resultId || !hasVehicle || !checks.length ||
      !$('measurement-reviewed').checked || !$('image-source-confirmed').checked;
    $('restore').disabled = busy || !canApply || (!active && !restoreToken);
    for(const id of ['columns','rows','mode','unit','cell-size','total-width','total-height','aligned','vehicle-unit','center-x','center-y','check-x','check-y','check-unit','measurement-reviewed','image-source-confirmed','ground-contact'])$(id).disabled=busy;
    for (const id of ['zoom-out', 'zoom-in', 'zoom', 'fit', 'pan', 'reset', 'live-again']) $(id).disabled = !frameId || busy;
    $('undo').disabled = !points.length || busy;
  }
  function parameters() {
    const mode = $('mode').value;
    const dimensions = mode === 'cell' ? {mode, unit: $('unit').value, cell_size: numeric('cell-size')} :
      {mode, unit: $('unit').value, width: numeric('total-width'), height: numeric('total-height')};
    const vehicle = $('aligned').checked ? {axes_aligned: true, unit: $('vehicle-unit').value, x: numeric('center-x'), y: numeric('center-y')} : {axes_aligned: false};
    return {columns: numeric('columns'), rows: numeric('rows'), dimensions, image_points: points.map(p => [...p]), vehicle};
  }
  function numeric(id) { return $(id).value === '' ? null : Number($(id).value); }
  function actualSize() {
    const cell = $('mode').value === 'cell', scale = $('unit').value === 'mm' ? .001 : .01;
    $('cell-fields').disabled = !cell; $('total-fields').disabled = cell;
    $('vehicle-fields').disabled = !$('aligned').checked;
    const n = numeric('columns'), m = numeric('rows');
    const w = (cell ? n * numeric('cell-size') : numeric('total-width')) * scale;
    const h = (cell ? m * numeric('cell-size') : numeric('total-height')) * scale;
    const valid = Number.isInteger(n) && n > 0 && Number.isInteger(m) && m > 0 && Number.isFinite(w) && w > 0 && Number.isFinite(h) && h > 0;
    const fmt = v => Number(v.toPrecision(8)).toString();
    $('actual').textContent = valid ? `最终采用：${fmt(w*1000)} × ${fmt(h*1000)} mm（${fmt(w)} × ${fmt(h)} m）` : '尺寸无效：请输入正整数格数和正的实际尺寸。';
  }
  function clearResult() {
    resultId = null; saved = false;
    checkMode = false; checkPoint = null; checks = []; hasVehicle = false;
    $('check-mode').setAttribute('aria-pressed','false'); $('checks').replaceChildren();
    $('check-coordinate').textContent='计算后可切换检查模式，检查点不会修改四个角点。';
    for (const id of ['measurement-reviewed','image-source-confirmed','ground-contact']) $(id).checked=false;
    $('new-ground').textContent = '角点或输入改变，请重新计算当前 H。';
    $('candidate-panel').hidden = true; $('preview-panel').hidden = true;
    $('bev').removeAttribute('src'); $('record').textContent = '';
    $('result-state').textContent = '尚未计算'; controls();
  }
  function cancelPending() { clearTimeout(autoTimer); clearTimeout(invalidateTimer); }
  function changed() {
    revision += 1; clearResult(); actualSize(); $('error').textContent = '';
    $('status').textContent = points.length < 4 ? `已选 ${points.length}/4 点，下一点：${names[points.length]}。` : '四角已完整，正在计算当前 H…';
    cancelPending();
    const id = frameId, r = revision;
    if (id) invalidateTimer = setTimeout(() => post('/invalidate', {frame_id: id, revision: r}).catch(() => {}), 60);
    if (id && points.length === 4) autoTimer = setTimeout(compute, 220);
    draw();
  }
  function draw() {
    const width = canvas.clientWidth, height = canvas.clientHeight, dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(width*dpr); canvas.height = Math.round(height*dpr);
    context.setTransform(dpr, 0, 0, dpr, 0, 0); context.fillStyle = '#152524'; context.fillRect(0, 0, width, height);
    if (!image) return;
    context.drawImage(image, view.x, view.y, image.naturalWidth*view.scale, image.naturalHeight*view.scale);
    context.strokeStyle = '#64efb0'; context.lineWidth = 2;
    context.beginPath(); points.forEach((p, i) => { const x=view.x+p[0]*view.scale,y=view.y+p[1]*view.scale; i ? context.lineTo(x,y) : context.moveTo(x,y); });
    if (points.length === 4) context.closePath(); context.stroke();
    points.forEach((p,i) => {
      const x=view.x+p[0]*view.scale,y=view.y+p[1]*view.scale;
      context.beginPath(); context.arc(x,y,10,0,Math.PI*2); context.fillStyle='#fff8d7'; context.fill();
      context.strokeStyle='#165d43'; context.stroke(); context.fillStyle='#18352c'; context.font='bold 13px system-ui'; context.textAlign='center';context.textBaseline='middle';context.fillText(String(i+1),x,y);
    });
    if(checkPoint){const x=view.x+checkPoint[0]*view.scale,y=view.y+checkPoint[1]*view.scale;
      context.strokeStyle='#ffcf64';context.lineWidth=2;context.beginPath();context.moveTo(x-9,y);context.lineTo(x+9,y);context.moveTo(x,y-9);context.lineTo(x,y+9);context.stroke();}
    $('points').replaceChildren(...points.map((p,i) => {const li=document.createElement('li');li.dataset.x=p[0];li.dataset.y=p[1];li.textContent=`${i+1} ${names[i]} · (${p[0].toFixed(2)}, ${p[1].toFixed(2)}) px`;return li;}));
  }
  function fit() {
    if (!image) return;
    fitScale = Math.min(canvas.clientWidth/image.naturalWidth, canvas.clientHeight/image.naturalHeight);
    view.scale = fitScale * Number($('zoom').value)/100;
    view.x = (canvas.clientWidth-image.naturalWidth*view.scale)/2;
    view.y = (canvas.clientHeight-image.naturalHeight*view.scale)/2; draw();
  }
  function pixel(event) {return clientToImage(event.clientX,event.clientY,canvas.getBoundingClientRect(),[canvas.clientWidth,canvas.clientHeight],view);}
  function zoom(value, event=null) {
    if (!image) return;
    const anchor = event ? pixel(event) : [(canvas.clientWidth/2-view.x)/view.scale,(canvas.clientHeight/2-view.y)/view.scale];
    const old = view.scale;
    $('zoom').value = Math.max(50,Math.min(600,value)); view.scale = fitScale*Number($('zoom').value)/100;
    view.x += anchor[0]*(old-view.scale); view.y += anchor[1]*(old-view.scale); draw();
  }
  canvas.addEventListener('pointerdown', event => {
    if (!frameId || !image || event.button > 0 || busy) return;
    const p = pixel(event), hit = points.findIndex(q => Math.hypot(q[0]-p[0],q[1]-p[1])*view.scale < 18);
    if(checkMode && !panMode){
      if(p[0]>=0 && p[1]>=0 && p[0]<image.naturalWidth && p[1]<image.naturalHeight){checkPoint=p;draw();inspectPoint(false);}
      event.preventDefault();return;
    }
    if (hit >= 0) drag = {kind:'point',index:hit};
    else if (panMode) drag = {kind:'pan',client:[event.clientX,event.clientY],start:[view.x,view.y]};
    else if (points.length < 4 && p[0]>=0 && p[1]>=0 && p[0]<image.naturalWidth && p[1]<image.naturalHeight) {points.push(p);changed();}
    canvas.setPointerCapture(event.pointerId);event.preventDefault();
  });
  canvas.addEventListener('pointermove', event => {
    if (!drag || !image) return;
    if (drag.kind === 'point') {const p=pixel(event);points[drag.index]=[Math.max(0,Math.min(image.naturalWidth-1,p[0])),Math.max(0,Math.min(image.naturalHeight-1,p[1]))];changed();}
    else {const rect=canvas.getBoundingClientRect();view.x=drag.start[0]+(event.clientX-drag.client[0])*canvas.clientWidth/rect.width;view.y=drag.start[1]+(event.clientY-drag.client[1])*canvas.clientHeight/rect.height;draw();}
  });
  for (const name of ['pointerup','pointercancel','lostpointercapture']) canvas.addEventListener(name,()=>{drag=null;});
  canvas.addEventListener('wheel',event=>{if(image){event.preventDefault();zoom(Number($('zoom').value)*(event.deltaY<0?1.15:1/1.15),event);}},{passive:false});
  $('zoom').oninput = () => zoom(Number($('zoom').value));
  $('zoom-in').onclick = () => zoom(Number($('zoom').value)*1.4);
  $('zoom-out').onclick = () => zoom(Number($('zoom').value)/1.4);
  $('fit').onclick = () => {$('zoom').value=100;fit();};
  $('pan').onclick = () => {panMode=!panMode;$('pan').setAttribute('aria-pressed',panMode);$('pan').textContent=panMode?'平移中 · 点击退出':'拖动画面';};
  $('undo').onclick = () => {points.pop();changed();};
  $('reset').onclick = () => {points=[];changed();};
  for (const id of ['columns','rows','mode','unit','cell-size','total-width','total-height','aligned','vehicle-unit','center-x','center-y']) $(id).addEventListener('input',changed);

  function clearFrame() {
    cancelPending(); frameId=null; points=[]; image=null; revision=0; drag=null;panMode=false;
    $('pan').setAttribute('aria-pressed','false');$('pan').textContent='拖动画面';
    clearResult(); $('points').replaceChildren();$('viewport').hidden=true; $('error').textContent='';
    $('status').textContent='等待新的冻结或上传原图，旧角点和结果已清除。';controls();
  }
  function showBaseline(value) {
    baseline=value;$('old-ground').textContent=value.error||matrix(value.H_img_to_vehicle_ground_m);
    $('old-bev').textContent=matrix(value.H_img_to_navigation_bev_px);
    $('baseline-note').textContent=`来源：启动时只读导航配置，原图 ${value.image_size.join(' × ')}；参数估算、未验收。`;
  }
  async function acceptFrame(data) {
    frameId=data.frame_id;revision=data.revision;showBaseline(data.baseline);
    const id=frameId,img=new Image();img.src='/frozen.png?id='+encodeURIComponent(id);
    await img.decode();if(id!==frameId)return;
    image=img;$('viewport').hidden=false;$('live-wrap').hidden=true;$('zoom').value=100;fit();controls();
    $('status').textContent=`原图 ${data.image_size.join(' × ')} px 已冻结，请选择区域左上外角。`;
    if (data.source.matches_navigation_resolution===false) $('baseline-note').textContent+=' 上传图与导航分辨率不同，标定前 H 不直接适用于此图。';
  }
  $('capture').onclick = async () => {
    clearFrame();busy=true;controls();
    try {await acceptFrame(await post('/capture',{}));}catch(error){$('error').textContent=error.message;}finally{busy=false;controls();}
  };
  $('upload').onchange = async () => {
    const file=$('upload').files[0];if(!file)return;clearFrame();busy=true;controls();
    try {const response=await fetch('/upload?filename='+encodeURIComponent(file.name),{method:'POST',headers:{'Content-Type':'application/octet-stream'},body:file});const data=await response.json();if(!response.ok)throw Error(data.message);await acceptFrame(data);}catch(error){$('error').textContent=error.message;}finally{busy=false;$('upload').value='';controls();}
  };
  $('live-again').onclick = () => {if(frameId)post('/invalidate',{frame_id:frameId,revision:revision+1}).catch(()=>{});clearFrame();$('live-wrap').hidden=false;$('status').textContent=offline?'请上传新的原始图片。':'正在显示实时画面，冻结后再选点。';};
  async function compute() {
    if (busy || !frameId || points.length!==4) return;
    clearTimeout(autoTimer);const id=frameId,r=revision,params=parameters();busy=true;controls();
    try {
      const data=await post('/compute',{frame_id:id,revision:r,parameters:params});
      if(id!==frameId || r!==revision)return;
      resultId=data.result_id;saved=false;const record=data.record;
      hasVehicle=!!record.vehicle_candidate;
      $('new-ground').textContent=matrix(record.H_img_to_region_ground_m);
      $('candidate-panel').hidden=!record.vehicle_candidate;
      if(record.vehicle_candidate)$('vehicle-ground').textContent=matrix(record.vehicle_candidate.H_img_to_vehicle_ground_m);
      $('bev').src='/bev.png?id='+encodeURIComponent(resultId);$('preview-panel').hidden=false;
      $('record').textContent=JSON.stringify(record,null,2);$('result-state').textContent='当前结果有效 · 尚未验收';
      $('status').textContent='四点拟合成功。核对实际宽高、角点方向和俯视预览后独立保存。';$('error').textContent='';
    }catch(error){if(id===frameId && r===revision){clearResult();$('error').textContent=error.message;$('status').textContent='本次输入无效，不能保存旧结果。';}}
    finally{busy=false;controls();if(id===frameId && r!==revision && points.length===4)autoTimer=setTimeout(compute,100);}
  }
  $('compute').onclick=compute;
  function currentValues(){return {frame_id:frameId,revision,result_id:resultId,parameters:parameters()};}
  function showApplication(value,error=null){
    active=value;
    $('application-badge').textContent=error?'已选标定异常':active?'已选手动标定':'未选手动标定';
    $('active-status').textContent=error||(!canApply?'临时输出目录：应用功能禁用，验证不会影响本项目导航。':active?`下次导航启动使用：${active.calibration_file}。${active.ground_contact_verified?'循迹和涵洞使用同一份标定。':'循迹可用；涵洞停车仍缺少底边地面接触确认。'}`:'下次导航启动使用原配置；涵洞沿用原有标定或显式估算模式。');
    $('active-ground').textContent=matrix(active?active.candidate.H_img_to_vehicle_ground_m:baseline?.H_img_to_vehicle_ground_m);
    controls();
  }
  $('check-mode').onclick=()=>{checkMode=!checkMode;$('check-mode').setAttribute('aria-pressed',checkMode);$('check-coordinate').textContent=checkMode?'检查模式：点击原图中的已知地面位置。':'选角模式：可以拖动四角，修改后检查记录会清除。';};
  async function inspectPoint(recordMeasured){
    if(!resultId||!checkPoint||busy)return;
    const id=resultId,r=revision;busy=true;controls();
    try{
      const values={...currentValues(),image_point:[...checkPoint]};
      if(recordMeasured)values.measured={unit:$('check-unit').value,x:numeric('check-x'),y:numeric('check-y')};
      const data=await post('/check-point',values);if(id!==resultId||r!==revision)return;
      const fmt=p=>p.map(v=>(v*100).toFixed(2)).join(', ');
      $('check-coordinate').textContent=`原图 (${data.image_point.map(v=>v.toFixed(2)).join(', ')}) px；区域 (X,Y)=(${fmt(data.region_ground_m)}) cm；`+(data.vehicle_ground_m?`车辆 (X,Y)=(${fmt(data.vehicle_ground_m)}) cm。`:'填写中心偏移后才能得到车辆距离。');
      checks=data.checks;$('checks').replaceChildren(...checks.map((check,i)=>{const li=document.createElement('li');li.textContent=`${i+1} 实测 (${fmt(check.measured_ground_m)}) cm，计算 (${fmt(check.predicted_ground_m)}) cm，误差 ΔX/ΔY=(${fmt(check.error_xy_m)}) cm，总误差 ${(check.error_m*100).toFixed(2)} cm`;return li;}));
      if(recordMeasured)$('measurement-reviewed').checked=false;
      $('error').textContent='';
    }catch(error){$('error').textContent=error.message;}finally{busy=false;controls();}
  }
  $('record-check').onclick=()=>inspectPoint(true);
  for(const id of ['measurement-reviewed','image-source-confirmed','ground-contact'])$(id).addEventListener('input',controls);
  $('apply').onclick=async()=>{
    busy=true;controls();
    try{const data=await post('/apply',{...currentValues(),measurement_reviewed:$('measurement-reviewed').checked,image_source_confirmed:$('image-source-confirmed').checked,ground_contact_verified:$('ground-contact').checked,expected_applied_at:active?.applied_at||null});saved=true;restoreToken=null;showApplication(data.active_calibration);$('status').textContent=data.message;$('error').textContent='';}
    catch(error){$('error').textContent=error.message;}finally{busy=false;controls();}
  };
  $('restore').onclick=async()=>{
    busy=true;controls();
    try{const data=await post('/restore',{expected_applied_at:active?.applied_at,restore_token:restoreToken});restoreToken=null;showApplication(data.active_calibration);$('status').textContent=data.message;$('error').textContent='';}
    catch(error){$('error').textContent=error.message;}finally{busy=false;controls();}
  };
  $('save').onclick=async()=>{
    const id=frameId,r=revision,result=resultId;busy=true;controls();
    try{const data=await post('/save',{frame_id:id,revision:r,result_id:result,parameters:parameters()});if(id===frameId&&r===revision){saved=true;$('status').textContent='已独立保存：'+data.path;}}
    catch(error){$('error').textContent=error.message;}finally{busy=false;controls();}
  };
  $('stop').onclick=async()=>{try{await post('/stop',{});clearFrame();canCapture=false;$('live').removeAttribute('src');$('status').textContent='网页已停止，相机正在释放。';controls();}catch(error){$('error').textContent=error.message;}};
  let initialized=false;
  async function poll() {
    try {
      const response=await fetch('/status'),data=await response.json();offline=data.offline;canCapture=data.can_capture;
      canApply=data.can_apply;if(!busy){restoreToken=data.restore_token;showApplication(data.active_calibration,data.application_error);}
      $('camera').textContent=`${data.offline?'离线上传模式':'导航相机 '+data.device} · ${data.navigation_image_size.join(' × ')} px`+(data.camera_error?' · '+data.camera_error:'');
      if(!initialized){initialized=true;showBaseline(data.baseline);if(!data.offline){$('live').src='/camera.mjpg';$('live-placeholder').hidden=true;}if(data.frame_id)await acceptFrame({frame_id:data.frame_id,revision:data.revision,image_size:data.frozen_image_size,source:data.frozen_source,baseline:data.baseline});}
      controls();
    }catch(error){$('camera').textContent='网页连接已断开';canCapture=false;controls();}
  }
  new ResizeObserver(()=>{if(image)fit();}).observe($('viewport'));
  actualSize();controls();poll();setInterval(poll,1500);
}
if (typeof document !== 'undefined') initPage();
