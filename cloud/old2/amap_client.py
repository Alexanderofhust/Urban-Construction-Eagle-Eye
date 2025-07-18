# Urban Construction Eagle Eye - 高德地图API客户端

import json
import logging
import requests
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class LocationInfo:
    """位置信息"""
    latitude: float
    longitude: float
    address: str = ""
    district: str = ""
    city: str = ""
    province: str = ""

@dataclass
class RouteInfo:
    """路径信息"""
    distance: float
    duration: int
    polyline: str
    steps: List[Dict[str, Any]]

class AMapClient:
    """高德地图API客户端"""
    
    def __init__(self, api_key: str, security_code: str = None):
        """
        初始化高德地图客户端
        
        Args:
            api_key: 高德地图API密钥
            security_code: 安全密钥（可选）
        """
        self.api_key = api_key
        self.security_code = security_code
        self.base_url = "https://restapi.amap.com/v3"
        self.session = requests.Session()
        
        # 设置请求头
        self.session.headers.update({
            'User-Agent': 'Urban-Construction-Eagle-Eye/1.0'
        })
        
        logger.info("高德地图API客户端初始化完成")
    
    def _make_request(self, endpoint: str, params: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        发送API请求
        
        Args:
            endpoint: API端点
            params: 请求参数
            
        Returns:
            API响应数据
        """
        try:
            # 添加基础参数
            params['key'] = self.api_key
            params['output'] = 'json'
            
            url = f"{self.base_url}/{endpoint}"
            
            logger.info(f"发送高德地图API请求: {endpoint}")
            response = self.session.get(url, params=params, timeout=30)
            response.raise_for_status()
            
            data = response.json()
            
            # 检查API响应状态
            if data.get('status') == '1':
                return data
            else:
                error_msg = data.get('info', '未知错误')
                logger.error(f"高德地图API错误: {error_msg}")
                return None
                
        except requests.exceptions.RequestException as e:
            logger.error(f"高德地图API请求失败: {str(e)}")
            return None
        except json.JSONDecodeError as e:
            logger.error(f"高德地图API响应解析失败: {str(e)}")
            return None
    
    def geocode(self, address: str, city: str = None) -> Optional[LocationInfo]:
        """
        地址解析（地址转坐标）
        
        Args:
            address: 地址
            city: 城市名称（可选）
            
        Returns:
            位置信息
        """
        try:
            params = {
                'address': address
            }
            
            if city:
                params['city'] = city
            
            data = self._make_request('geocode/geo', params)
            
            if data and data.get('geocodes'):
                geocode = data['geocodes'][0]
                location = geocode['location'].split(',')
                
                return LocationInfo(
                    latitude=float(location[1]),
                    longitude=float(location[0]),
                    address=geocode.get('formatted_address', address),
                    district=geocode.get('district', ''),
                    city=geocode.get('city', ''),
                    province=geocode.get('province', '')
                )
            
            return None
            
        except Exception as e:
            logger.error(f"地址解析失败: {str(e)}")
            return None
    
    def reverse_geocode(self, latitude: float, longitude: float) -> Optional[LocationInfo]:
        """
        逆地址解析（坐标转地址）
        
        Args:
            latitude: 纬度
            longitude: 经度
            
        Returns:
            位置信息
        """
        try:
            params = {
                'location': f"{longitude},{latitude}",
                'poitype': '',
                'radius': 1000,
                'extensions': 'all',
                'roadlevel': 0
            }
            
            data = self._make_request('geocode/regeo', params)
            
            if data and data.get('regeocode'):
                regeocode = data['regeocode']
                address_component = regeocode.get('addressComponent', {})
                
                return LocationInfo(
                    latitude=latitude,
                    longitude=longitude,
                    address=regeocode.get('formatted_address', ''),
                    district=address_component.get('district', ''),
                    city=address_component.get('city', ''),
                    province=address_component.get('province', '')
                )
            
            return None
            
        except Exception as e:
            logger.error(f"逆地址解析失败: {str(e)}")
            return None
    
    def get_district_bounds(self, district_name: str) -> Optional[Dict[str, Any]]:
        """
        获取行政区域边界
        
        Args:
            district_name: 行政区域名称
            
        Returns:
            区域边界信息
        """
        try:
            params = {
                'keywords': district_name,
                'subdistrict': 0,
                'extensions': 'all'
            }
            
            data = self._make_request('config/district', params)
            
            if data and data.get('districts'):
                district = data['districts'][0]
                return {
                    'name': district.get('name'),
                    'center': district.get('center'),
                    'level': district.get('level'),
                    'polyline': district.get('polyline'),
                    'bounds': self._parse_bounds(district.get('polyline', ''))
                }
            
            return None
            
        except Exception as e:
            logger.error(f"获取行政区域边界失败: {str(e)}")
            return None
    
    def _parse_bounds(self, polyline: str) -> List[List[float]]:
        """
        解析区域边界坐标
        
        Args:
            polyline: 边界坐标字符串
            
        Returns:
            坐标点列表
        """
        try:
            if not polyline:
                return []
            
            bounds = []
            polygons = polyline.split('|')
            
            for polygon in polygons:
                if not polygon:
                    continue
                    
                points = []
                coords = polygon.split(';')
                
                for coord in coords:
                    if coord:
                        lng, lat = coord.split(',')
                        points.append([float(lng), float(lat)])
                
                if points:
                    bounds.append(points)
            
            return bounds
            
        except Exception as e:
            logger.error(f"解析区域边界失败: {str(e)}")
            return []
    
    def search_nearby_pois(self, latitude: float, longitude: float, 
                          keywords: str = "", radius: int = 1000) -> List[Dict[str, Any]]:
        """
        搜索附近POI
        
        Args:
            latitude: 纬度
            longitude: 经度
            keywords: 搜索关键词
            radius: 搜索半径（米）
            
        Returns:
            POI列表
        """
        try:
            params = {
                'location': f"{longitude},{latitude}",
                'keywords': keywords,
                'radius': radius,
                'extensions': 'all'
            }
            
            data = self._make_request('place/around', params)
            
            if data and data.get('pois'):
                pois = []
                for poi in data['pois']:
                    location = poi.get('location', '').split(',')
                    if len(location) == 2:
                        pois.append({
                            'id': poi.get('id'),
                            'name': poi.get('name'),
                            'address': poi.get('address'),
                            'longitude': float(location[0]),
                            'latitude': float(location[1]),
                            'distance': poi.get('distance'),
                            'type': poi.get('type')
                        })
                return pois
            
            return []
            
        except Exception as e:
            logger.error(f"搜索附近POI失败: {str(e)}")
            return []
    
    def get_route_directions(self, origin: Tuple[float, float], 
                           destination: Tuple[float, float], 
                           strategy: int = 0) -> Optional[RouteInfo]:
        """
        获取路径规划
        
        Args:
            origin: 起点坐标 (longitude, latitude)
            destination: 终点坐标 (longitude, latitude)
            strategy: 路径策略 (0:速度优先, 1:费用优先, 2:距离优先)
            
        Returns:
            路径信息
        """
        try:
            params = {
                'origin': f"{origin[0]},{origin[1]}",
                'destination': f"{destination[0]},{destination[1]}",
                'strategy': strategy,
                'extensions': 'all'
            }
            
            data = self._make_request('direction/driving', params)
            
            if data and data.get('route'):
                route = data['route']
                paths = route.get('paths', [])
                
                if paths:
                    path = paths[0]
                    return RouteInfo(
                        distance=float(path.get('distance', 0)),
                        duration=int(path.get('duration', 0)),
                        polyline=path.get('polyline', ''),
                        steps=path.get('steps', [])
                    )
            
            return None
            
        except Exception as e:
            logger.error(f"获取路径规划失败: {str(e)}")
            return None
    
    def convert_coordinates(self, coordinates: List[Tuple[float, float]], 
                          from_type: str = "gps", to_type: str = "amap") -> List[Tuple[float, float]]:
        """
        坐标转换
        
        Args:
            coordinates: 坐标列表
            from_type: 源坐标系 (gps, mapbar, baidu, autonavi)
            to_type: 目标坐标系 (autonavi, mapbar, baidu)
            
        Returns:
            转换后的坐标列表
        """
        try:
            if not coordinates:
                return []
            
            # 构建坐标字符串
            locations = []
            for coord in coordinates:
                locations.append(f"{coord[0]},{coord[1]}")
            
            params = {
                'locations': '|'.join(locations),
                'coordsys': from_type,
                'output': to_type
            }
            
            data = self._make_request('assistant/coordinate/convert', params)
            
            if data and data.get('locations'):
                converted = []
                for location in data['locations'].split(';'):
                    if location:
                        lng, lat = location.split(',')
                        converted.append((float(lng), float(lat)))
                return converted
            
            return coordinates  # 如果转换失败，返回原坐标
            
        except Exception as e:
            logger.error(f"坐标转换失败: {str(e)}")
            return coordinates
    
    def get_static_map_url(self, center: Tuple[float, float], zoom: int = 10, 
                          size: str = "400*300", markers: List[Dict[str, Any]] = None) -> str:
        """
        获取静态地图URL
        
        Args:
            center: 地图中心点坐标 (longitude, latitude)
            zoom: 缩放级别
            size: 地图尺寸 "width*height"
            markers: 标记点列表
            
        Returns:
            静态地图URL
        """
        try:
            params = {
                'location': f"{center[0]},{center[1]}",
                'zoom': zoom,
                'size': size,
                'scale': 1,
                'key': self.api_key
            }
            
            # 添加标记点
            if markers:
                marker_strs = []
                for marker in markers:
                    marker_str = f"mid,0xFF0000,A:{marker['longitude']},{marker['latitude']}"
                    if marker.get('label'):
                        marker_str += f":{marker['label']}"
                    marker_strs.append(marker_str)
                params['markers'] = '|'.join(marker_strs)
            
            # 构建URL
            url = "https://restapi.amap.com/v3/staticmap?"
            url += "&".join([f"{k}={v}" for k, v in params.items()])
            
            return url
            
        except Exception as e:
            logger.error(f"获取静态地图URL失败: {str(e)}")
            return ""
    
    def get_weather_info(self, city: str) -> Optional[Dict[str, Any]]:
        """
        获取天气信息
        
        Args:
            city: 城市名称或城市编码
            
        Returns:
            天气信息
        """
        try:
            params = {
                'city': city,
                'extensions': 'all'
            }
            
            data = self._make_request('weather/weatherInfo', params)
            
            if data and data.get('forecasts'):
                forecast = data['forecasts'][0]
                return {
                    'city': forecast.get('city'),
                    'province': forecast.get('province'),
                    'report_time': forecast.get('reporttime'),
                    'casts': forecast.get('casts', [])
                }
            
            return None
            
        except Exception as e:
            logger.error(f"获取天气信息失败: {str(e)}")
            return None
    
    def create_js_api_config(self) -> Dict[str, Any]:
        """
        创建前端JS API配置
        
        Returns:
            JS API配置
        """
        config = {
            'key': self.api_key,
            'version': '2.0',
            'plugins': [
                'AMap.Scale',
                'AMap.ToolBar',
                'AMap.MapType',
                'AMap.Geolocation',
                'AMap.Marker',
                'AMap.InfoWindow',
                'AMap.Polyline',
                'AMap.Polygon',
                'AMap.Circle'
            ]
        }
        
        if self.security_code:
            config['security_code'] = self.security_code
        
        return config
    
    def validate_api_key(self) -> bool:
        """
        验证API密钥有效性
        
        Returns:
            是否有效
        """
        try:
            # 使用简单的地址解析请求来验证密钥
            data = self._make_request('geocode/geo', {'address': '北京市'})
            return data is not None
            
        except Exception as e:
            logger.error(f"验证API密钥失败: {str(e)}")
            return False
