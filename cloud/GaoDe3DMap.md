高德3D地图demo
```html
<!DOCTYPE HTML>
<html>
<head>
<meta name="viewport" content="width=device-width initial-scale=1.0 maximum-scale=1.0 user-scalable=0">
<meta http-equiv="Content-Type" content="text/html; charset=utf-8">
<title>3D地图</title>
<link rel="stylesheet" type="text/css" href="./index.css"/>
      <!-- 重要，请到 https://console.amap.com 申请 JS API 的 key和密钥 -->
      <script> 
        window._AMapSecurityConfig = {
          securityJsCode: '290ddc4b0d33be7bc9b354bc6a4ca614',
        }
      </script>
      
<script language="javascript" src="//webapi.amap.com/maps?v=2.0&key=6f025e700cbacbb0bb866712d20bb35c&plugin=AMap.ControlBar,AMap.ToolBar"></script>
</head>
<body >
<div id="container" style="width:100%; height:100%;resize:both;"></div>
<script language="javascript">
var map;
function mapInit(){
  map = new AMap.Map('container', {
    rotateEnable:true,
    pitchEnable:true,
    zoom: 17,
    pitch: 50,
    rotation: -15,
    viewMode:'3D', //开启3D视图,默认为关闭
    zooms:[2,20],
    center:[116.333926,39.997245]
  });
  
  var controlBar = new AMap.ControlBar({
    position:{
      right:'10px',
      top:'10px'
    }
  });
  controlBar.addTo(map);
  
  var toolBar = new AMap.ToolBar({
    position:{
      right:'40px',
      top:'110px'
    }
  });
  toolBar.addTo(map);
}
mapInit()
</script>
</body>
</html>
```