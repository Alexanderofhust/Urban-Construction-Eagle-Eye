import requests

class BaiduMapService:
    def __init__(self, ak):
        self.ak = ak  # 百度地图API密钥
        self.base_url = "https://api.map.baidu.com"

    def geocode(self, address):
        """将地址转换为经纬度（用于建筑位置定位）"""
        url = f"{self.base_url}/geocoding/v3/?address={address}&output=json&ak={self.ak}"
        res = requests.get(url).json()
        if res["status"] == 0:
            location = res["result"]["location"]
            return (location["lat"], location["lng"])  # 纬度，经度
        else:
            raise ValueError(f"地址解析失败：{res['msg']}")

    def create_map_marker(self, building_id, lat, lng, analysis_result):
        """生成地图标记数据（供前端JS渲染）"""
        # 标记信息：包含建筑ID、经纬度、受损率、点云查看链接
        return {
            "building_id": building_id,
            "location": (lat, lng),
            "info": f"受损率：{analysis_result['damaged_rate']:.2%}",
            "pcd_view_url": f"/view_pcd?building_id={building_id}",  # 点云查看页面链接
            "analysis_url": f"/view_analysis?building_id={building_id}"  # 分析结果页面链接
        }