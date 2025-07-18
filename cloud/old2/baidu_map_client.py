"""
百度地图客户端
功能：
1. 与百度地图API交互
2. 地理坐标转换
3. 地图可视化支持
"""

import requests
import json
import logging
from typing import Dict, List, Tuple, Optional
import os

logger = logging.getLogger(__name__)

class BaiduMapClient:
    """百度地图客户端"""
    
    def __init__(self, ak: Optional[str] = None):
        self.ak = ak or os.getenv('BAIDU_MAP_AK', 'your_baidu_map_ak')
        self.base_url = "https://api.map.baidu.com"
        
        # 坐标系转换URL
        self.coord_convert_url = f"{self.base_url}/geoconv/v1/"
        
        # 地理编码URL
        self.geocoding_url = f"{self.base_url}/geocoding/v3/"
        
        # 逆地理编码URL
        self.reverse_geocoding_url = f"{self.base_url}/reverse_geocoding/v3/"
    
    def convert_coordinates(self, coords: List[Tuple[float, float]], 
                          from_coord: str = "wgs84ll", 
                          to_coord: str = "bd09ll") -> List[Tuple[float, float]]:
        """
        坐标系转换
        from_coord: 源坐标系 (wgs84ll, gcj02ll, bd09ll)
        to_coord: 目标坐标系 (wgs84ll, gcj02ll, bd09ll)
        """
        try:
            # 构造坐标字符串
            coords_str = ";".join([f"{lon},{lat}" for lat, lon in coords])
            
            params = {
                'coords': coords_str,
                'from': from_coord,
                'to': to_coord,
                'ak': self.ak
            }
            
            response = requests.get(self.coord_convert_url, params=params, timeout=10)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('status') == 0:
                    converted_coords = []
                    for result in data.get('result', []):
                        converted_coords.append((result['y'], result['x']))  # lat, lon
                    return converted_coords
                else:
                    logger.error(f"坐标转换失败: {data.get('message', 'Unknown error')}")
                    return coords
            else:
                logger.error(f"API请求失败: {response.status_code}")
                return coords
                
        except Exception as e:
            logger.error(f"坐标转换异常: {str(e)}")
            return coords
    
    def get_address_info(self, lat: float, lon: float) -> Dict:
        """根据坐标获取地址信息"""
        try:
            params = {
                'location': f"{lat},{lon}",
                'output': 'json',
                'coordtype': 'bd09ll',
                'ak': self.ak
            }
            
            response = requests.get(self.reverse_geocoding_url, params=params, timeout=10)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('status') == 0:
                    result = data.get('result', {})
                    return {
                        'formatted_address': result.get('formatted_address', ''),
                        'business': result.get('business', ''),
                        'city': result.get('addressComponent', {}).get('city', ''),
                        'district': result.get('addressComponent', {}).get('district', ''),
                        'street': result.get('addressComponent', {}).get('street', ''),
                        'street_number': result.get('addressComponent', {}).get('street_number', ''),
                        'pois': result.get('pois', [])
                    }
                else:
                    logger.error(f"地址查询失败: {data.get('message', 'Unknown error')}")
                    return {}
            else:
                logger.error(f"API请求失败: {response.status_code}")
                return {}
                
        except Exception as e:
            logger.error(f"地址查询异常: {str(e)}")
            return {}
    
    def get_coordinates_from_address(self, address: str) -> Optional[Tuple[float, float]]:
        """根据地址获取坐标"""
        try:
            params = {
                'address': address,
                'output': 'json',
                'ak': self.ak
            }
            
            response = requests.get(self.geocoding_url, params=params, timeout=10)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('status') == 0:
                    result = data.get('result', {})
                    location = result.get('location', {})
                    lat = location.get('lat')
                    lng = location.get('lng')
                    
                    if lat is not None and lng is not None:
                        return (lat, lng)
                    else:
                        logger.error("坐标信息不完整")
                        return None
                else:
                    logger.error(f"地理编码失败: {data.get('message', 'Unknown error')}")
                    return None
            else:
                logger.error(f"API请求失败: {response.status_code}")
                return None
                
        except Exception as e:
            logger.error(f"地理编码异常: {str(e)}")
            return None
    
    def create_map_config(self, center_lat: float, center_lng: float, 
                         markers: List[Dict] = None, 
                         zoom: int = 15) -> Dict:
        """创建地图配置"""
        try:
            config = {
                'center': {'lat': center_lat, 'lng': center_lng},
                'zoom': zoom,
                'enableScrollWheelZoom': True,
                'enableKeyboard': True,
                'enableDragging': True,
                'enableDoubleClickZoom': True,
                'mapType': 'BMAP_NORMAL_MAP',  # 普通地图
                'markers': markers or [],
                'ak': self.ak
            }
            
            return config
            
        except Exception as e:
            logger.error(f"创建地图配置失败: {str(e)}")
            return {}
    
    def create_building_marker(self, lat: float, lng: float, 
                             building_info: Dict) -> Dict:
        """创建建筑物标记"""
        try:
            # 根据缺陷严重程度选择标记颜色
            severity_colors = {
                'critical': '#FF0000',  # 红色
                'high': '#FF8000',      # 橙色
                'medium': '#FFFF00',    # 黄色
                'low': '#00FF00',       # 绿色
                'minimal': '#0000FF'    # 蓝色
            }
            
            severity = building_info.get('severity', 'medium')
            color = severity_colors.get(severity, '#FFFF00')
            
            marker = {
                'position': {'lat': lat, 'lng': lng},
                'icon': {
                    'url': f'data:image/svg+xml;base64,{self._create_marker_svg(color)}',
                    'size': {'width': 30, 'height': 30}
                },
                'title': building_info.get('name', '建筑物'),
                'infoWindow': {
                    'content': self._create_info_window_content(building_info),
                    'width': 300,
                    'height': 200
                }
            }
            
            return marker
            
        except Exception as e:
            logger.error(f"创建建筑物标记失败: {str(e)}")
            return {}
    
    def _create_marker_svg(self, color: str) -> str:
        """创建SVG标记图标"""
        svg_content = f'''
        <svg xmlns="http://www.w3.org/2000/svg" width="30" height="30">
            <circle cx="15" cy="15" r="12" fill="{color}" stroke="#000" stroke-width="2"/>
            <text x="15" y="20" text-anchor="middle" font-size="12" fill="#000">●</text>
        </svg>
        '''
        
        import base64
        return base64.b64encode(svg_content.encode('utf-8')).decode('utf-8')
    
    def _create_info_window_content(self, building_info: Dict) -> str:
        """创建信息窗口内容"""
        content = f'''
        <div style="padding: 10px;">
            <h3 style="margin: 0 0 10px 0; color: #333;">
                {building_info.get('name', '建筑物')}
            </h3>
            <p><strong>地址:</strong> {building_info.get('address', '未知')}</p>
            <p><strong>检测时间:</strong> {building_info.get('detection_time', '未知')}</p>
            <p><strong>严重程度:</strong> 
                <span style="color: {self._get_severity_color(building_info.get('severity', 'medium'))};">
                    {self._get_severity_text(building_info.get('severity', 'medium'))}
                </span>
            </p>
            <p><strong>缺陷统计:</strong></p>
            <ul style="margin: 5px 0; padding-left: 20px;">
        '''
        
        defects = building_info.get('defects', {})
        for defect_type, count in defects.items():
            if count > 0:
                content += f'<li>{defect_type}: {count}</li>'
        
        content += '''
            </ul>
            <div style="margin-top: 10px;">
                <button onclick="viewDetails()" style="
                    background: #4CAF50; 
                    color: white; 
                    border: none; 
                    padding: 5px 10px; 
                    border-radius: 3px; 
                    cursor: pointer;
                ">查看详情</button>
            </div>
        </div>
        '''
        
        return content
    
    def _get_severity_color(self, severity: str) -> str:
        """获取严重程度颜色"""
        colors = {
            'critical': '#FF0000',
            'high': '#FF8000',
            'medium': '#FFFF00',
            'low': '#00FF00',
            'minimal': '#0000FF'
        }
        return colors.get(severity, '#FFFF00')
    
    def _get_severity_text(self, severity: str) -> str:
        """获取严重程度文本"""
        texts = {
            'critical': '严重',
            'high': '较严重',
            'medium': '中等',
            'low': '轻微',
            'minimal': '良好'
        }
        return texts.get(severity, '中等')
    
    def calculate_distance(self, lat1: float, lng1: float, 
                          lat2: float, lng2: float) -> float:
        """计算两点间距离（米）"""
        try:
            import math
            
            # 地球半径（米）
            R = 6371000
            
            # 转换为弧度
            lat1_rad = math.radians(lat1)
            lng1_rad = math.radians(lng1)
            lat2_rad = math.radians(lat2)
            lng2_rad = math.radians(lng2)
            
            # 计算差值
            dlat = lat2_rad - lat1_rad
            dlng = lng2_rad - lng1_rad
            
            # Haversine公式
            a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlng/2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            
            distance = R * c
            return distance
            
        except Exception as e:
            logger.error(f"计算距离失败: {str(e)}")
            return 0.0
    
    def get_building_bounds(self, coords: List[Tuple[float, float]]) -> Dict:
        """获取建筑物边界"""
        try:
            if not coords:
                return {}
            
            lats = [coord[0] for coord in coords]
            lngs = [coord[1] for coord in coords]
            
            return {
                'southwest': {'lat': min(lats), 'lng': min(lngs)},
                'northeast': {'lat': max(lats), 'lng': max(lngs)},
                'center': {'lat': sum(lats) / len(lats), 'lng': sum(lngs) / len(lngs)}
            }
            
        except Exception as e:
            logger.error(f"获取建筑物边界失败: {str(e)}")
            return {}
    
    def validate_coordinates(self, lat: float, lng: float) -> bool:
        """验证坐标有效性"""
        try:
            # 检查纬度范围
            if not (-90 <= lat <= 90):
                return False
            
            # 检查经度范围
            if not (-180 <= lng <= 180):
                return False
            
            return True
            
        except Exception as e:
            logger.error(f"坐标验证失败: {str(e)}")
            return False
