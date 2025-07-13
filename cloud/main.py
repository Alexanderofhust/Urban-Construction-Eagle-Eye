from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pcd_cal import PointCloudAnalyzer  # 导入用户提供的点云分析类
from map import BaiduMapService
from llm_client import LLMClient
import yaml
import os
from pathlib import Path

app = FastAPI()

# 允许跨域（前端调用后端API）
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"]
)

# 初始化服务（实际部署时从配置文件读取密钥）
baidu_map = BaiduMapService(ak="你的百度地图AK")
llm = LLMClient(api_key="你的大模型API密钥", base_url="https://api.openai.com/v1")

# 创建结果存储目录
Path("data/analysis_results").mkdir(parents=True, exist_ok=True)


@app.post("/analyze_pcd")
def analyze_pcd(
        pcd_path: str,
        building_id: str,
        address: str,
        r_min: float = 0.6,  # 红色点阈值（默认值）
        g_max: float = 0.8,
        b_max: float = 0.8
):
    """
    上传点云并分析（关联建筑地址和ID）
    - 红色点云视为"受损点云"，通过阈值控制检测逻辑
    """
    # 1. 初始化点云分析器并设置阈值
    analyzer = PointCloudAnalyzer()
    analyzer.set_red_threshold(r_min=r_min, g_max=g_max, b_max=b_max)  # 调整红色点检测阈值

    # 2. 执行点云分析（提取红色点云并生成结果）
    analysis_result = analyzer.analyze(pcd_path)
    if not analysis_result:
        raise HTTPException(status_code=400, detail="点云分析失败（可能无颜色信息或文件错误）")

    # 3. 补充建筑元信息（ID、地址）并关联地图坐标
    lat, lng = baidu_map.geocode(address)  # 将地址转换为经纬度
    full_result = {
        **analysis_result,  # 包含total_points、red_points_count、red_percentage等
        "building_id": building_id,
        "address": address,
        "location": (lat, lng),  # 经纬度坐标
        "damaged_rate": analysis_result["red_percentage"] / 100.0  # 转换为小数形式（与后续接口兼容）
    }

    # 4. 保存完整结果到YAML（供后续接口读取）
    yaml_path = f"data/analysis_results/{building_id}.yaml"
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(full_result, f, sort_keys=False, allow_unicode=True)

    # 5. 返回包含地图坐标的分析结果
    return full_result


@app.get("/get_map_markers")
def get_map_markers(building_ids: list[str]):
    """获取多个建筑的地图标记数据（用于前端百度地图渲染）"""
    markers = []
    for bid in building_ids:
        # 读取YAML中的分析结果
        yaml_path = f"data/analysis_results/{bid}.yaml"
        if not os.path.exists(yaml_path):
            raise HTTPException(status_code=404, detail=f"建筑ID {bid} 的分析结果不存在")

        with open(yaml_path, "r", encoding="utf-8") as f:
            result = yaml.safe_load(f)

        # 生成地图标记（包含建筑ID、位置、受损率、点云查看链接）
        markers.append(baidu_map.create_map_marker(
            building_id=bid,
            lat=result["location"][0],  # 纬度
            lng=result["location"][1],  # 经度
            analysis_result=result
        ))
    return markers


@app.get("/get_maintenance_route")
def get_maintenance_route(building_ids: list[str]):
    """调用大语言模型生成维修保养推荐路线"""
    # 收集所有建筑的关键信息（用于生成路线）
    building_info_list = []
    for bid in building_ids:
        yaml_path = f"data/analysis_results/{bid}.yaml"
        if not os.path.exists(yaml_path):
            raise HTTPException(status_code=404, detail=f"建筑ID {bid} 的分析结果不存在")

        with open(yaml_path, "r", encoding="utf-8") as f:
            res = yaml.safe_load(f)

        building_info_list.append({
            "building_id": bid,
            "address": res["address"],
            "damaged_rate": res["damaged_rate"],  # 受损率（小数）
            "location": res["location"]  # 经纬度
        })

    # 调用大语言模型生成路线（优先高受损率+地理最优）
    route = llm.generate_maintenance_route(building_info_list)
    return {"route": route}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)