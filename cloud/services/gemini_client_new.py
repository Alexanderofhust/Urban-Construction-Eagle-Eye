import google.generativeai as genai
import os
import logging
from typing import Dict, Optional, List
import asyncio
import json
from PIL import Image
import io
import base64

logger = logging.getLogger(__name__)

class GeminiClient:
    def __init__(self):
        # 从环境变量获取API密钥
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            logger.warning("GEMINI_API_KEY not found in environment variables")
            self.model = None
        else:
            genai.configure(api_key=api_key)
            self.model = genai.GenerativeModel('gemini-1.5-flash')
        
    async def analyze_building(self, images: Dict, session_id: str) -> Dict:
        """使用Gemini分析建筑
        
        Args:
            images: 包含以下键的字典:
                - 'pointcloud': 点云全景图的base64编码
                - 'defect_images': 最严重的2张缺陷图像的base64编码列表
            session_id: 会话ID
        """
        try:
            if not self.model:
                return self._create_default_analysis("Gemini API未配置")
            
            # 准备图像数据
            image_objects = []
            
            # 添加点云全景图
            if 'pointcloud' in images:
                pointcloud_data = base64.b64decode(images['pointcloud'])
                pointcloud_image = Image.open(io.BytesIO(pointcloud_data))
                image_objects.append(pointcloud_image)
            
            # 添加缺陷图像
            if 'defect_images' in images:
                for defect_img_b64 in images['defect_images']:
                    defect_data = base64.b64decode(defect_img_b64)
                    defect_image = Image.open(io.BytesIO(defect_data))
                    image_objects.append(defect_image)
            
            # 构建详细的分析提示词
            prompt = """
你是一位经验丰富的建筑工程师和质量检测专家。请分析以下图像：
- 第1张图像：建筑物的点云全景图，显示整体结构
- 第2-3张图像：检测出的最严重缺陷图像

请按以下结构进行详细分析：

1. **建筑物识别**
   - 建筑类型（住宅、商业、工业、桥梁、道路等）
   - 建筑结构形式（框架、剪力墙、砌体、钢结构等）
   - 主要建材类型（混凝土、钢材、砖石、木材等）
   - 建筑年代和设计特征

2. **缺陷识别与分类**
   - 缺陷具体类型（裂缝、剥落、变形、腐蚀、渗漏等）
   - 缺陷位置和分布特征
   - 缺陷严重程度评估（轻微/中等/严重/危险）
   - 缺陷发展趋势预测

3. **成因分析**
   - 设计因素（设计不当、标准变化等）
   - 施工因素（工艺问题、材料质量等）
   - 使用因素（超载、使用不当等）
   - 环境因素（老化、自然灾害、气候影响等）

4. **风险评估**
   - 结构安全风险等级
   - 可能影响的功能
   - 潜在的连锁反应
   - 紧急处理需求

5. **维修建议**
   - 立即处理措施
   - 中期维修方案
   - 长期预防策略
   - 推荐的检测频率

6. **预防改善措施**
   - 设计改进建议
   - 施工质量控制要点
   - 日常维护要求
   - 监测系统建议

请以JSON格式返回分析结果，包含以下字段：
{
    "building_identification": {
        "building_type": "建筑类型",
        "structure_type": "结构形式",
        "materials": ["主要建材"],
        "estimated_age": "估计年代"
    },
    "defect_analysis": {
        "defect_types": ["缺陷类型列表"],
        "severity_level": "严重程度",
        "locations": ["缺陷位置"],
        "development_trend": "发展趋势"
    },
    "root_causes": ["可能成因列表"],
    "risk_assessment": {
        "safety_risk": "安全风险等级",
        "functional_impact": ["功能影响"],
        "urgency": "紧急程度"
    },
    "repair_recommendations": {
        "immediate_actions": ["立即措施"],
        "medium_term": ["中期方案"], 
        "long_term": ["长期策略"],
        "inspection_frequency": "检测频率"
    },
    "prevention_measures": {
        "design_improvements": ["设计改进"],
        "construction_controls": ["施工控制"],
        "maintenance_requirements": ["维护要求"],
        "monitoring_suggestions": ["监测建议"]
    },
    "overall_assessment": {
        "condition_rating": "整体状况评级",
        "condition_score": 8.5,
        "priority_level": "优先级"
    },
    "detailed_summary": "详细分析总结"
}
"""
            
            # 调用Gemini API
            content = [prompt] + image_objects
            response = await asyncio.get_event_loop().run_in_executor(
                None,
                self.model.generate_content,
                content
            )
            
            # 解析响应
            result_text = response.text
            logger.info(f"Gemini响应原文: {result_text[:500]}...")
            
            # 尝试解析JSON
            try:
                # 提取JSON部分
                json_start = result_text.find('{')
                json_end = result_text.rfind('}') + 1
                if json_start != -1 and json_end != -1:
                    json_str = result_text[json_start:json_end]
                    result = json.loads(json_str)
                else:
                    # 如果没有找到JSON，创建默认结构
                    result = self._create_default_analysis(result_text)
                    
            except json.JSONDecodeError as e:
                logger.warning(f"JSON解析失败: {e}")
                result = self._create_default_analysis(result_text)
            
            # 添加元数据
            result['session_id'] = session_id
            result['analysis_timestamp'] = asyncio.get_event_loop().time()
            result['image_count'] = len(image_objects)
            
            logger.info(f"建筑分析完成，会话ID: {session_id}")
            return result
            
        except Exception as e:
            logger.error(f"建筑分析失败: {str(e)}")
            return self._create_default_analysis(f"分析失败: {str(e)}")
    
    def _create_default_analysis(self, error_msg: str = "") -> Dict:
        """创建默认分析结果"""
        return {
            "building_identification": {
                "building_type": "未识别",
                "structure_type": "需要人工确认",
                "materials": ["需要现场确认"],
                "estimated_age": "待评估"
            },
            "defect_analysis": {
                "defect_types": ["需要专业检测"],
                "severity_level": "待评估",
                "locations": ["需要现场标记"],
                "development_trend": "需要持续监测"
            },
            "root_causes": ["需要详细调查"],
            "risk_assessment": {
                "safety_risk": "待评估",
                "functional_impact": ["需要专业评估"],
                "urgency": "建议尽快检查"
            },
            "repair_recommendations": {
                "immediate_actions": ["建议联系专业工程师"],
                "medium_term": ["制定详细检测计划"],
                "long_term": ["建立定期监测机制"],
                "inspection_frequency": "每月一次"
            },
            "prevention_measures": {
                "design_improvements": ["需要专业评估"],
                "construction_controls": ["严格按规范施工"],
                "maintenance_requirements": ["定期维护保养"],
                "monitoring_suggestions": ["安装监测设备"]
            },
            "overall_assessment": {
                "condition_rating": "需要专业评估",
                "condition_score": 5.0,
                "priority_level": "中等"
            },
            "detailed_summary": f"自动分析结果有限，建议人工专业检查。{error_msg}",
            "error_message": error_msg
        }
    
    async def test_connection(self) -> bool:
        """测试API连接"""
        try:
            if not self.model:
                return False
            
            # 简单测试
            response = await asyncio.get_event_loop().run_in_executor(
                None,
                self.model.generate_content,
                "测试连接"
            )
            return True
            
        except Exception as e:
            logger.error(f"Gemini连接测试失败: {str(e)}")
            return False
