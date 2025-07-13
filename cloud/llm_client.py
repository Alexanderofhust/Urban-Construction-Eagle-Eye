import requests


class LLMClient:
    def __init__(self, api_key, base_url):
        self.api_key = api_key
        self.base_url = base_url  # 大语言模型API地址（如OpenAI：https://api.openai.com/v1/chat/completions）

    def generate_maintenance_route(self, building_info_list):
        """
        生成维修保养推荐路线
        building_info_list：建筑信息列表，格式如：
        [
            {"building_id": "A1", "address": "北京市天安门", "damaged_rate": 0.3, "location": (39.9148, 116.4038)},
            ...
        ]
        """
        # 构造prompt
        prompt = f"""
        请根据以下建筑的受损情况和位置，推荐维修保养路线：
        1. 优先维修受损率>20%的建筑；
        2. 路线需按地理距离最短规划，避免绕路；
        3. 输出格式：路线顺序（建筑ID+地址）+ 推荐理由。

        建筑信息：
        {building_info_list}
        """
        # 调用大语言模型API（以OpenAI为例）
        headers = {"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"}
        data = {
            "model": "gpt-3.5-turbo",
            "messages": [{"role": "user", "content": prompt}]
        }
        res = requests.post(f"{self.base_url}/chat/completions", json=data, headers=headers).json()
        return res["choices"][0]["message"]["content"]