高德鼠標事件歷程。
```html
<!doctype html>
<html>
<head>
    <meta charset="utf-8">
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta name="viewport" content="initial-scale=1.0, user-scalable=no, width=device-width">
    <title>覆盖物事件</title>
    <link rel="stylesheet" href="https://a.amap.com/jsapi_demos/static/demo-center/css/demo-center.css"/>
    <style>
        html,body,#container{
            height:100%;
            width:100%;
        }
    </style>
</head>
<body>
<div id="container"></div>
<div class="info" id="text">
    请点击覆盖物试试
</div>
<div class="input-card" style="width:18rem">
    <h4>覆盖物点击事件的绑定与解绑</h4>
    <div>
      <div class="input-item">
        <button id="clickOn" class="btn" style="margin-right:1rem;">绑定事件</button>
        <button id="clickOff" class="btn">解绑事件</button>
      </div>
    </div>
</div>
<script type="text/javascript" src="https://webapi.amap.com/maps?v=2.0&key=您申请的key值"></script>
<script src="https://a.amap.com/jsapi_demos/static/demo-center/js/demoutils.js"></script>
<script type="text/javascript">
    //初始化地图对象，加载地图
    var map = new AMap.Map("container", {
        resizeEnable: true
    });
    var marker = new AMap.Marker({
        map: map,
        icon: "https://webapi.amap.com/theme/v1.3/markers/n/mark_b.png",
        position: [116.405467, 39.907761]
    });
    var lineArr = [
        [116.368904, 39.913423],
        [116.382122, 39.901176],
        [116.387271, 39.912501],
        [116.398258, 39.904600]
    ];
    var circle = new AMap.Circle({
        map: map,
        center: lineArr[0],          //设置线覆盖物路径
        radius: 1500,
        strokeColor: "#3366FF", //边框线颜色
        strokeOpacity: 0.3,       //边框线透明度
        strokeWeight: 3,        //边框线宽
        fillColor: "#FFA500", //填充色
        fillOpacity: 0.35//填充透明度
    });
    var polygonArr = [[116.403322, 39.920255],
        [116.410703, 39.897555],
        [116.402292, 39.892353],
        [116.389846, 39.891365]];
    var polygon = new AMap.Polygon({
        map: map,
        path: polygonArr,//设置多边形边界路径
        strokeColor: "#FF33FF", //线颜色
        strokeOpacity: 0.2, //线透明度
        strokeWeight: 3,    //线宽
        fillColor: "#1791fc", //填充色
        fillOpacity: 0.35//填充透明度
    });
    map.setFitView();

    function showInfoM(e){
        var text = '您在 [ '+e.lnglat.getLng()+','+e.lnglat.getLat()+' ] 的位置点击了marker！'
        document.querySelector("#text").innerText = text;
    }
    function showInfoC(e){
        var text = '您在 [ '+e.lnglat.getLng()+','+e.lnglat.getLat()+' ] 的位置点击了圆！'
        document.querySelector("#text").innerText = text;
    }
    function showInfoP(e){
        var text = '您在 [ '+e.lnglat.getLng()+','+e.lnglat.getLat()+' ] 的位置点击了多边形！'
        document.querySelector("#text").innerText = text;
    }
    function showInfoOver(e){
        var text = '鼠标移入覆盖物！'
        document.querySelector("#text").innerText = text;
    }
    function showInfoOut(e){
        var text = '鼠标移出覆盖物！'
        document.querySelector("#text").innerText = text;
    }
    
    function clickOn(){
        log.success("绑定事件!");  

        marker.on('click', showInfoM);
        circle.on('click', showInfoC);
        polygon.on('click', showInfoP);

        marker.on('mouseover', showInfoOver);
        circle.on('mouseover', showInfoOver);
        polygon.on('mouseover', showInfoOver);

        marker.on('mouseout', showInfoOut);
        circle.on('mouseout', showInfoOut);
        polygon.on('mouseout', showInfoOut);
    }
    function clickOff(){
        log.success("解除事件绑定!"); 

        marker.off('click', showInfoM);
        circle.off('click', showInfoC);
        polygon.off('click', showInfoP);

        marker.off('mouseover', showInfoOver);
        circle.off('mouseover', showInfoOver);
        polygon.off('mouseover', showInfoOver);

        marker.off('mouseout', showInfoOut);
        circle.off('mouseout', showInfoOut);
        polygon.off('mouseout', showInfoOut);
    }
    
    // 给按钮绑定事件
    document.getElementById("clickOn").onclick = clickOn;
    document.getElementById("clickOff").onclick = clickOff;
</script>
</body>
</html>
```