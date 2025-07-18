# 高德地图API集成指南

## 概述

本系统已从百度地图API切换到高德地图API，提供更稳定和功能丰富的地图服务。

## 配置步骤

### 1. 获取高德地图API密钥

1. 访问 [高德开放平台](https://lbs.amap.com/)
2. 注册账号并完成实名认证
3. 创建应用：
   - 选择"Web服务"类型
   - 填写应用名称和描述
   - 选择合适的服务类型

4. 获取API密钥：
   - 在控制台找到你的应用
   - 复制"Web服务"的API密钥

5. 申请安全密钥（重要）：
   - 2021年12月02日后申请的密钥需要配合安全密钥使用
   - 在应用详情页面申请JS API安全密钥

### 2. 配置环境变量

在 `.env` 文件中添加：

```env
# 高德地图API密钥
AMAP_API_KEY=your_amap_api_key
AMAP_SECURITY_CODE=your_amap_security_code
```

### 3. 配置域名白名单

在高德开放平台的应用设置中：
1. 找到"域名白名单"设置
2. 添加你的域名，例如：
   - `localhost:8000`
   - `your-domain.com`

## 功能特性

### 地图基础功能
- 2D/3D地图展示
- 多种地图样式（标准、卫星、路网等）
- 地图缩放、平移、旋转
- 地图控件（比例尺、工具栏、地图类型切换）

### 位置服务
- 地理编码（地址转坐标）
- 逆地理编码（坐标转地址）
- IP定位
- 浏览器定位

### 标记和覆盖物
- 标记点（Marker）
- 信息窗口（InfoWindow）
- 折线（Polyline）
- 多边形（Polygon）
- 圆形（Circle）
- 矩形（Rectangle）

### 路径规划
- 驾车路径规划
- 公交路径规划
- 步行路径规划
- 骑行路径规划

### 坐标转换
- GPS坐标转换
- 不同坐标系间转换
- 批量坐标转换

## API使用示例

### 基础地图初始化

```javascript
// 设置安全密钥
window._AMapSecurityConfig = {
    securityJsCode: 'your_security_code'
};

// 加载高德地图API
AMapLoader.load({
    key: 'your_api_key',
    version: '2.0',
    plugins: ['AMap.Scale', 'AMap.ToolBar']
}).then((AMap) => {
    // 创建地图实例
    const map = new AMap.Map('mapContainer', {
        center: [116.404, 39.915],
        zoom: 15
    });
    
    // 添加控件
    map.addControl(new AMap.Scale());
    map.addControl(new AMap.ToolBar());
});
```

### 添加标记点

```javascript
// 创建标记点
const marker = new AMap.Marker({
    position: [116.404, 39.915],
    title: '标记点标题'
});

// 添加到地图
map.add(marker);

// 创建信息窗口
const infoWindow = new AMap.InfoWindow({
    content: '<div>这是信息窗口内容</div>'
});

// 标记点点击事件
marker.on('click', () => {
    infoWindow.open(map, marker.getPosition());
});
```

### 地理编码

```javascript
// 地址转坐标
AMap.plugin('AMap.Geocoder', () => {
    const geocoder = new AMap.Geocoder();
    
    geocoder.getLocation('北京市朝阳区阜通东大街6号', (status, result) => {
        if (status === 'complete' && result.geocodes.length) {
            const location = result.geocodes[0].location;
            console.log('坐标:', location.lng, location.lat);
        }
    });
});
```

### 路径规划

```javascript
// 驾车路径规划
AMap.plugin('AMap.Driving', () => {
    const driving = new AMap.Driving({
        map: map,
        panel: 'route-panel'
    });
    
    driving.search([
        { lng: 116.379028, lat: 39.865042 },
        { lng: 116.427281, lat: 39.903719 }
    ], (status, result) => {
        if (status === 'complete') {
            console.log('路径规划成功');
        }
    });
});
```

## 系统集成

### 后端API客户端

系统提供了完整的Python客户端 `AMapClient`，支持：

```python
from amap_client import AMapClient

# 初始化客户端
client = AMapClient(api_key='your_key', security_code='your_code')

# 地理编码
location = client.geocode('北京市朝阳区')

# 逆地理编码
address = client.reverse_geocode(39.915, 116.404)

# 搜索附近POI
pois = client.search_nearby_pois(39.915, 116.404, '餐厅')

# 路径规划
route = client.get_route_directions((116.379, 39.865), (116.427, 39.903))
```

### 前端集成

前端自动获取API配置：

```javascript
// 获取地图配置
async function getMapConfig() {
    const response = await fetch('/api/map/config');
    const result = await response.json();
    return result.data;
}

// 初始化地图
async function initMap() {
    const config = await getMapConfig();
    // 使用配置初始化地图
}
```

## 常见问题

### 1. 地图不显示
- 检查API密钥是否正确
- 确认域名白名单是否配置
- 检查网络连接

### 2. 安全密钥错误
- 确保使用最新的安全密钥
- 检查密钥是否匹配正确的应用

### 3. 配额限制
- 检查API调用量是否超限
- 升级套餐或优化调用频率

### 4. 坐标系问题
- 高德地图使用GCJ-02坐标系
- 需要时进行坐标转换

## 性能优化

### 1. 地图加载优化
- 按需加载插件
- 使用CDN加速
- 启用地图缓存

### 2. 标记点优化
- 使用聚合标记
- 懒加载标记数据
- 优化标记图标

### 3. 数据传输优化
- 压缩数据传输
- 使用WebSocket推送
- 缓存常用数据

## 升级说明

### 从百度地图迁移

1. **API调用差异**：
   - 百度地图：`new BMap.Map()`
   - 高德地图：`new AMap.Map()`

2. **坐标系差异**：
   - 百度地图：BD-09坐标系
   - 高德地图：GCJ-02坐标系

3. **事件处理**：
   - 事件名称略有不同
   - 参数格式可能不同

4. **插件加载**：
   - 高德地图采用按需加载
   - 更灵活的插件管理

### 兼容性处理

系统已提供完整的兼容性处理，原有功能无需修改即可使用。

## 技术支持

如有问题，可以：
1. 查阅高德地图官方文档
2. 访问开发者社区
3. 联系技术支持

相关链接：
- [高德开放平台](https://lbs.amap.com/)
- [JS API文档](https://lbs.amap.com/api/jsapi-v2/summary)
- [开发者社区](https://lbs.amap.com/community)
