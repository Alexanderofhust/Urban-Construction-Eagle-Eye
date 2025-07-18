JS API 的加载
最后更新时间: 2023年12月18日
JS API 2.0 版本提供了两种方案引入：

1. 使用 JS API Loader 进行加载；

2. 以<script>标签的方式加载；

注意
为避免地图数据协议和前端资源不匹配导致页面运行报错，只允许在线加载 JS API，禁止进行本地转存、与其它代码混合打包等用法。

1、准备
成为开发者并创建 key
为了正常调用 API ，请先注册成为高德开放平台开发者，并申请 web 平台（JS API）的 key 和安全密钥，点击 具体操作。

提示
你在2021年12月02日以后申请的 key 需要配合你的安全密钥一起使用。

2、使用 JS API Loader (推荐)
JS API Loader 是我们提供的 API 加载器，具有以下特性：

支持以 <script>标签和 npm 包两种方式使用；
有效避免错误异步加载导致的 JS API 资源加载不完整问题；
对于加载混用多个版本 JS API 的错误用法给予报错处理；
对于不合法加载引用 JS API 给予报错处理；
支持指定 JS API 版本；
支持插件加载；
允许多次执行加载操作，网络资源不会重复请求，便于大型工程模块管理；
支持 IE9 及以上的浏览器
2.1 <script> 标签加载 Loader 
使用<script>标签加载 loader.js

<script src="https://webapi.amap.com/loader.js"></script>
<script type="text/javascript">
  window._AMapSecurityConfig = {
    securityJsCode: "「你申请的安全密钥」",
  };
  AMapLoader.load({
    key: "替换为你申请的 key", //申请好的 Web 端开发 Key，首次调用 load 时必填
    version: "2.0", //指定要加载的 JS API 的版本，缺省时默认为 1.4.15
    plugins: ["AMap.Scale"], //需要使用的的插件列表，如比例尺'AMap.Scale'，支持添加多个如：['AMap.Scale','...','...']
    AMapUI: {
      //是否加载 AMapUI，缺省不加载
      version: "1.1", //AMapUI 版本
      plugins: ["overlay/SimpleMarker"], //需要加载的 AMapUI ui 插件
    },
    Loca: {
      //是否加载 Loca， 缺省不加载
      version: "2.0", //Loca 版本
    },
  })
    .then((AMap) => {
      var map = new AMap.Map("container"); //"container"为 <div> 容器的 id
      map.addControl(new AMap.Scale()); //添加比例尺组件到地图实例上
    })
    .catch((e) => {
      console.error(e); //加载错误提示
    });
</script>
JavaScript
 JS API示例  AMapUI示例  Loca示例 
2.2 NPM 安装 Loader
安装：

npm i @amap/amap-jsapi-loader --save
shell
 使用 NPM 安装 Loader，并设置 key 和安全密钥：

import AMapLoader from "@amap/amap-jsapi-loader";
window._AMapSecurityConfig = {
  securityJsCode: "「你申请的安全密钥」",
};
AMapLoader.load({
  key: "替换为你申请的 key", //申请好的 Web 端开发者 Key，首次调用 load 时必填
  version: "2.0", //指定要加载的 JS API 的版本，缺省时默认为 1.4.15
  plugins: ["AMap.Scale"], //需要使用的的插件列表，如比例尺'AMap.Scale'，支持添加多个如：['AMap.Scale','...','...']
})
  .then((AMap) => {
    var map = new AMap.Map("container"); //"container"为 <div> 容器的 id
  })
  .catch((e) => {
    console.log(e);
  });
JavaScript
JS API 结合 Vue 使用JS API 结合 React 使用
3、<script> 标签加载 JS API 脚本
3.1 同步加载
<script> 标签加载 JS API 2.0

<!-- 需要设置元素的宽高样式 -->
<div id="container"></div>
<script type="text/javascript">
  window._AMapSecurityConfig = {
    securityJsCode: "「你申请的安全密钥」",
  };
</script>
<script
  type="text/javascript"
  src="https://webapi.amap.com/maps?v=2.0&key=你申请的key值"
></script>
<script type="text/javascript">
  //地图初始化应该在地图容器 <div> 已经添加到 DOM 树之后
  var map = new AMap.Map("container", {
    zoom: 12,
  });
</script>
HTML
3.2 异步加载
特别注意
异步加载指的是通过appendChild()将 JS API 的<script>标签添加到页面中，或者给<script>标签添加了async属性的使用方式。这种使用场景下，JS API 不会阻塞页面其他内容的执行和解析，但是 JS API 的脚本解析将有可能发生其他脚本资源执行之后，因为需要特别处理，以保证在 AMap 对象完整生成之后再调用 JS API 的相关接口，否则有可能报错。


提示
异步回调函数的声明应该在 JS API 引入之前。


使用appendChild()：

<script>
  //设置你的安全密钥
  window._AMapSecurityConfig = {
    securityJsCode: "「你申请的安全密钥」",
  };
  //声明异步加载回调函数
  window.onLoad = function () {
    var map = new AMap.Map("container"); //"container"为<div>容器的id
  };
  var url ="https://webapi.amap.com/maps?v=2.0&key=你申请的key值&callback=onLoad";
  var jsapi = document.createElement("script");
  jsapi.charset = "utf-8";
  jsapi.src = url;
  document.head.appendChild(jsapi);
</script>
JavaScript
使用 async 属性：

<script type="text/javascript"> //声明异步加载回调函数
  //设置你的安全密钥
  window._AMapSecurityConfig = {
    securityJsCode: "「你申请的安全密钥」",
  };
  window.onLoad = function () {
    var map = new AMap.Map("container");
  };
</script>
<script
  src="https://webapi.amap.com/maps?v=2.0&key=你申请的key值&callback=onLoad"
  async="async"
  type="text/javascript"
></script>