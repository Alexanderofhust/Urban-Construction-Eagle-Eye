"""
Gemini AI客户端
功能：
1. 连接到Google Gemini API
2. 分析建筑缺陷
3. 生成分析报告和建议
"""

import google.generativeai as genai
import logging
from typing import Dict, List, Optional, Any
import base64
import json
from PIL import Image
import io
import os

logger = logging.getLogger(__name__)

class GeminiClient:
    """Gemini AI客户端"""
    
    def __init__(self, api_key: Optional[str] = None):
        self.api_key = api_key or os.getenv('GEMINI_API_KEY')
        self.model = None
        self.initialized = False
        
        if self.api_key:
            self.initialize()
    
    def initialize(self):
        """初始化Gemini客户端"""
        try:
            genai.configure(api_key=self.api_key)
            self.model = genai.GenerativeModel('gemini-1.5-flash')
            self.initialized = True
            logger.info("Gemini客户端初始化成功")
            
        except Exception as e:
            logger.error(f"Gemini客户端初始化失败: {str(e)}")
            self.initialized = False
    
    async def analyze_building_defects(self, image_path: str, analysis_type: str = "building_defect") -> Dict[str, Any]:
        """分析建筑缺陷"""
        try:
            if not self.initialized:
                return {"error": "Gemini客户端未初始化"}
            
            logger.info(f"开始AI分析: {image_path}")
            
            # 加载图像
            image = Image.open(image_path)
            
            # 构造分析提示
            prompt = self._build_analysis_prompt(analysis_type)
            
            # 调用Gemini API
            response = self.model.generate_content([prompt, image])
            
            # 解析响应
            analysis_result = self._parse_analysis_response(response.text)
            
            logger.info("AI分析完成")
            return analysis_result
            
        except Exception as e:
            logger.error(f"AI分析失败: {str(e)}")
            return {"error": str(e)}
    
    def _build_analysis_prompt(self, analysis_type: str) -> str:
        """构建分析提示"""
        
        base_prompt = """
        你是一个专业的建筑结构工程师和缺陷检测专家。请分析这个建筑物的3D点云图像，识别潜在的结构缺陷和问题。
        
        请提供以下分析：
        
        1. 建筑物整体描述：
           - 建筑物类型和结构特征
           - 大致尺寸和规模
           - 建筑材料推测
        
        2. 缺陷检测和分析：
           - 识别图像中的红色、蓝色、绿色标记区域
           - 每种颜色代表的缺陷类型分析
           - 缺陷的严重程度评估
           - 缺陷的空间分布特征
        
        3. 结构安全评估：
           - 整体结构完整性评估
           - 潜在的安全隐患
           - 结构承载能力影响分析
        
        4. 维修建议：
           - 优先维修的区域和项目
           - 维修方法和材料建议
           - 预计维修时间和成本估算
           - 临时安全措施建议
        
        5. 监控建议：
           - 需要重点监控的区域
           - 监控频率建议
           - 预警指标设置
        
        请以JSON格式返回分析结果，包含以下字段：
        {
            "building_description": "建筑物描述",
            "defect_analysis": {
                "red_defects": {"type": "缺陷类型", "severity": "严重程度", "count": 数量},
                "blue_defects": {"type": "缺陷类型", "severity": "严重程度", "count": 数量},
                "green_defects": {"type": "缺陷类型", "severity": "严重程度", "count": 数量}
            },
            "safety_assessment": {
                "overall_rating": "整体评级(1-5)",
                "risk_level": "风险等级",
                "critical_areas": ["关键区域列表"]
            },
            "repair_recommendations": [
                {
                    "priority": "优先级",
                    "area": "维修区域",
                    "method": "维修方法",
                    "estimated_cost": "预计成本",
                    "timeline": "时间安排"
                }
            ],
            "monitoring_plan": {
                "key_areas": ["重点监控区域"],
                "frequency": "监控频率",
                "indicators": ["预警指标"]
            }
        }
        """
        
        if analysis_type == "structural_analysis":
            base_prompt += """
            
            特别关注：
            - 结构变形和位移
            - 裂缝发展趋势
            - 连接节点状况
            - 承重构件完整性
            """
        
        elif analysis_type == "maintenance_planning":
            base_prompt += """
            
            特别关注：
            - 维修优先级排序
            - 成本效益分析
            - 维修资源需求
            - 施工可行性
            """
        
        return base_prompt
    
    def _parse_analysis_response(self, response_text: str) -> Dict[str, Any]:
        """解析AI分析响应"""
        try:
            # 尝试提取JSON部分
            start_index = response_text.find('{')
            end_index = response_text.rfind('}') + 1
            
            if start_index >= 0 and end_index > start_index:
                json_text = response_text[start_index:end_index]
                analysis_result = json.loads(json_text)
                
                # 添加原始响应
                analysis_result['raw_response'] = response_text
                
                return analysis_result
            else:
                # 如果无法提取JSON，返回原始文本
                return {
                    "error": "无法解析AI响应",
                    "raw_response": response_text,
                    "building_description": "AI分析响应格式错误",
                    "defect_analysis": {},
                    "safety_assessment": {
                        "overall_rating": "Unknown",
                        "risk_level": "Unknown",
                        "critical_areas": []
                    },
                    "repair_recommendations": [],
                    "monitoring_plan": {
                        "key_areas": [],
                        "frequency": "Unknown",
                        "indicators": []
                    }
                }
                
        except json.JSONDecodeError as e:
            logger.error(f"JSON解析失败: {str(e)}")
            return {
                "error": f"JSON解析失败: {str(e)}",
                "raw_response": response_text
            }
        except Exception as e:
            logger.error(f"解析AI响应失败: {str(e)}")
            return {
                "error": f"解析失败: {str(e)}",
                "raw_response": response_text
            }
    
    def analyze_text_only(self, defect_data: Dict[str, Any]) -> Dict[str, Any]:
        """基于文本数据分析（不需要图像）"""
        try:
            if not self.initialized:
                return {"error": "Gemini客户端未初始化"}
            
            # 构建文本提示
            prompt = f"""
            基于以下建筑缺陷检测数据，请提供专业的分析和建议：
            
            缺陷统计数据：
            {json.dumps(defect_data, indent=2, ensure_ascii=False)}
            
            请提供：
            1. 缺陷严重程度评估
            2. 结构安全建议
            3. 维修优先级排序
            4. 预防措施建议
            
            请以JSON格式返回分析结果。
            """
            
            response = self.model.generate_content(prompt)
            return self._parse_analysis_response(response.text)
            
        except Exception as e:
            logger.error(f"文本分析失败: {str(e)}")
            return {"error": str(e)}
    
    def generate_report(self, analysis_data: Dict[str, Any]) -> str:
        """生成分析报告"""
        try:
            if not self.initialized:
                return "Gemini客户端未初始化"
            
            prompt = f"""
            基于以下建筑缺陷分析数据，生成一份专业的建筑缺陷检测报告：
            
            {json.dumps(analysis_data, indent=2, ensure_ascii=False)}
            
            报告应包含：
            1. 执行摘要
            2. 检测方法和设备
            3. 缺陷发现和分析
            4. 安全评估
            5. 维修建议
            6. 结论和建议
            
            请用专业的工程术语撰写，格式清晰，内容详实。
            """
            
            response = self.model.generate_content(prompt)
            return response.text
            
        except Exception as e:
            logger.error(f"生成报告失败: {str(e)}")
            return f"生成报告失败: {str(e)}"
    
    def get_repair_cost_estimate(self, defect_analysis: Dict[str, Any]) -> Dict[str, Any]:
        """获取维修成本估算"""
        try:
            if not self.initialized:
                return {"error": "Gemini客户端未初始化"}
            
            prompt = f"""
            基于以下建筑缺陷分析，请提供详细的维修成本估算：
            
            {json.dumps(defect_analysis, indent=2, ensure_ascii=False)}
            
            请提供：
            1. 各类缺陷的维修成本估算
            2. 材料成本明细
            3. 人工成本估算
            4. 设备租赁费用
            5. 总体预算范围
            6. 成本优化建议
            
            请以JSON格式返回，包含具体的数字估算。
            """
            
            response = self.model.generate_content(prompt)
            return self._parse_analysis_response(response.text)
            
        except Exception as e:
            logger.error(f"成本估算失败: {str(e)}")
            return {"error": str(e)}
    
    def check_api_status(self) -> Dict[str, Any]:
        """检查API状态"""
        try:
            if not self.api_key:
                return {"status": "error", "message": "API密钥未设置"}
            
            if not self.initialized:
                return {"status": "error", "message": "客户端未初始化"}
            
            # 发送简单的测试请求
            response = self.model.generate_content("请回复'API正常'")
            
            if response and response.text:
                return {"status": "ok", "message": "API连接正常"}
            else:
                return {"status": "error", "message": "API响应异常"}
                
        except Exception as e:
            logger.error(f"API状态检查失败: {str(e)}")
            return {"status": "error", "message": str(e)}
