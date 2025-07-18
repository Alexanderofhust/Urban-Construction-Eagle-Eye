// 3D地图管理器
class Map3DManager {
    constructor() {
        this.map3d = null;
        this.sessionOverlays = [];  // 存储所有会话的覆盖物
        this.sessionsData = [];     // 存储所有会话数据
        this.infoWindow = null;     // 悬浮信息窗口
        this.currentHoverSession = null;  // 当前悬浮的会话
        
        // 演示数据 - 保留用于测试
        this.demoLocationData = {
            session_id: 'demo_session',
            created_time: new Date().toISOString(),
            center: { lat: 39.90923, lon: 116.397428 },
            bounds: {
                southwest: { lat: 39.90800, lon: 116.39600 },
                northeast: { lat: 39.91046, lon: 116.39886 }
            },
            points: [
                { lat: 39.90850, lon: 116.39650 },
                { lat: 39.90950, lon: 116.39680 },
                { lat: 39.90980, lon: 116.39780 },
                { lat: 39.90920, lon: 116.39820 },
                { lat: 39.90860, lon: 116.39790 },
                { lat: 39.90830, lon: 116.39720 },
                { lat: 39.90850, lon: 116.39650 }
            ],
            total_points: 7,
            image_count: 50,
            mask_count: 25,
            has_pointcloud: true
        };
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
        
        // 等待高德地图API加载完成
        if (window.AMap) {
            console.log('AMap已加载，立即初始化地图');
            this.initMap3D();
        } else {
            console.log('AMap未加载，等待amapLoaded事件');
            window.addEventListener('amapLoaded', () => {
                console.log('接收到amapLoaded事件，开始初始化地图');
                this.initMap3D();
            });
            
            // 添加超时检查
            setTimeout(() => {
                if (!this.map3d && window.AMap) {
                    console.log('超时检查：AMap已存在但地图未初始化，尝试初始化');
                    this.initMap3D();
                }
            }, 3000);
        }
    }
    
    setupEventListeners() {
        // 使用事件委托来处理按钮点击
        document.addEventListener('click', (e) => {
            if (e.target.id === 'load-all-sessions-btn') {
                console.log('点击了加载所有会话按钮');
                this.loadAllSessions();
            } else if (e.target.id === 'reload-sessions-btn') {
                console.log('点击了重新加载会话按钮');
                this.reloadAllSessions();
            } else if (e.target.id === 'toggle-effects-btn') {
                console.log('点击了切换特效按钮');
                this.toggleSessionOverlays();
            } else if (e.target.id === 'reset-view-btn') {
                console.log('点击了重置视角按钮');
                this.reset3DView();
            } else if (e.target.id === 'load-map3d-btn') {
                console.log('点击了加载演示数据按钮');
                this.loadDemoData();
            }
        });
    }
    
    // 初始化3D地图
    initMap3D() {
        const mapContainer = document.getElementById('map3d-canvas');
        if (!mapContainer) {
            console.log('地图容器未找到，可能是页面未切换到3D地图页面');
            return;
        }
        
        // 检查容器是否可见
        const mapPage = document.getElementById('map3d-page');
        if (mapPage && !mapPage.classList.contains('active')) {
            console.log('3D地图页面未激活，暂不初始化地图');
            return;
        }
        
        console.log('地图容器找到:', mapContainer);
        console.log('地图容器尺寸:', mapContainer.offsetWidth, 'x', mapContainer.offsetHeight);
        
        if (!window.AMap) {
            console.error('AMap未加载，等待加载完成...');
            // 如果AMap还没加载完成，等待加载完成事件
            window.addEventListener('amapLoaded', () => {
                console.log('接收到amapLoaded事件，重新尝试初始化地图');
                this.initMap3D();
            });
            return;
        }
        
        if (this.map3d) {
            console.log('地图已存在，无需重复初始化');
            return;
        }
        
        try {
            console.log('开始创建3D地图实例...');
            console.log('AMap版本:', window.AMap.version);
            
            this.map3d = new AMap.Map('map3d-canvas', {
                rotateEnable: true,
                pitchEnable: true,
                zoom: 16,
                pitch: 45,
                rotation: -15,
                viewMode: '3D', // 开启3D视图
                zooms: [3, 20],
                center: [116.397428, 39.90923], // 北京天安门
                mapStyle: 'amap://styles/normal',
                resizeEnable: true,
                showBuildingBlock: true, // 显示建筑物
                features: ['bg', 'road', 'building', 'point']
            });
            
            console.log('3D地图创建成功');
            console.log('地图中心点:', this.map3d.getCenter());
            console.log('地图缩放级别:', this.map3d.getZoom());
            
            // 隐藏加载指示器
            const loadingIndicator = document.getElementById('map3d-loading');
            if (loadingIndicator) {
                loadingIndicator.style.display = 'none';
            }
            
        } catch (error) {
            console.error('创建地图失败:', error);
            
            // 显示错误信息
            const mapContainer = document.getElementById('map3d-canvas');
            if (mapContainer) {
                mapContainer.innerHTML = `
                    <div style="display: flex; align-items: center; justify-content: center; height: 100%; color: #fff; flex-direction: column; text-align: center; padding: 20px;">
                        <h3>🗺️ 地图初始化失败</h3>
                        <p style="margin: 10px 0;">错误信息: ${error.message}</p>
                        <p style="margin: 10px 0;">请检查网络连接或稍后重试</p>
                        <button onclick="window.location.reload()" style="margin-top: 10px; padding: 10px 20px; cursor: pointer; background: #007cba; color: white; border: none; border-radius: 4px;">重新加载</button>
                    </div>
                `;
            }
            return;
        }
        
        // 添加3D控制栏
        try {
            const controlBar = new AMap.ControlBar({
                position: {
                    right: '10px',
                    top: '10px'
                }
            });
            controlBar.addTo(this.map3d);
            console.log('3D控制栏添加成功');
        } catch (error) {
            console.error('添加3D控制栏失败:', error);
        }
        
        // 添加工具栏
        try {
            const toolBar = new AMap.ToolBar({
                position: {
                    right: '40px',
                    top: '110px'
                }
            });
            toolBar.addTo(this.map3d);
            console.log('工具栏添加成功');
        } catch (error) {
            console.error('添加工具栏失败:', error);
        }
        
        // 添加比例尺
        try {
            const scale = new AMap.Scale({
                position: {
                    bottom: '10px',
                    left: '10px'
                }
            });
            this.map3d.addControl(scale);
            console.log('比例尺添加成功');
        } catch (error) {
            console.error('添加比例尺失败:', error);
        }
        
        // 地图加载完成后自动加载演示数据
        this.map3d.on('complete', () => {
            console.log('3D地图加载完成事件触发，开始加载演示数据');
            setTimeout(() => {
                this.loadDemoData();
            }, 1000);
        });
        
        // 监听视角变化
        this.map3d.on('mapmove', () => {
            this.updateViewInfo();
        });
        
        // 立即尝试加载演示数据（不等待complete事件）
        setTimeout(() => {
            console.log('立即尝试加载演示数据');
            this.loadDemoData();
        }, 2000);
        
        console.log('3D地图初始化完成');
    }
    
    // 加载演示数据
    loadDemoData() {
        console.log('loadDemoData被调用');
        
        if (!this.map3d) {
            console.error('地图未初始化，尝试先初始化地图');
            this.initMap3D();
            if (!this.map3d) {
                console.error('地图初始化失败，无法加载演示数据');
                return;
            }
        }
        
        console.log('开始加载演示数据');
        console.log('演示数据:', this.demoLocationData);
        
        try {
            // 先设置地图中心和视角
            this.map3d.setCenter([116.397428, 39.90923]);
            this.map3d.setZoom(16);
            this.map3d.setPitch(45);
            this.map3d.setRotation(-15);
            
            console.log('地图视角已设置');
            
            // 然后更新地图数据和特效
            this.updateMapWithLocation(this.demoLocationData);
            this.createBuildingHighlight(this.demoLocationData);
            this.updateInfoPanel(this.demoLocationData);
            
            console.log('演示数据加载完成');
            
        } catch (error) {
            console.error('加载演示数据时出错:', error);
        }
    }
    
    // 加载会话数据
    async loadSessionData(sessionId) {
        try {
            const response = await fetch(`/api/location/${sessionId}`);
            const data = await response.json();
            
            if (data.location) {
                //console.log('加载会话数据成功:', data.location);
                console.log(`加载会话数据成功，位置数据: ${JSON.stringify(data.location)}`);
                this.updateMapWithLocation(data.location);
                this.createBuildingHighlight(data.location);
                this.updateInfoPanel(data.location);
            }
        } catch (error) {
            console.error('加载位置数据失败:', error);
            // 如果加载失败，使用演示数据
            this.loadDemoData();
        }
    }

    // 加载所有会话数据并显示在地图上
    async loadAllSessions() {
        console.log('开始加载所有会话数据');
        
        if (!this.map3d) {
            console.error('地图未初始化，无法加载会话数据');
            return;
        }

        try {
            // 显示加载状态
            this.updateStatus('正在加载所有会话数据...');
            
            // 获取会话列表
            const sessionsResponse = await fetch('/api/sessions');
            const sessionsData = await sessionsResponse.json();
            
            if (!sessionsData.sessions || sessionsData.sessions.length === 0) {
                console.log('没有找到会话数据');
                this.updateStatus('没有找到会话数据，加载演示数据');
                this.loadDemoData();
                return;
            }

            console.log(`找到 ${sessionsData.sessions.length} 个会话`);
            this.sessionsData = [];
            
            // 清除现有覆盖物
            this.clearAllSessionOverlays();
            
            // 为每个会话加载位置数据
            for (const session of sessionsData.sessions) {
                try {
                    const locationResponse = await fetch(`/api/location/${session.session_id}`);
                    const locationData = await locationResponse.json();
                    
                    if (locationData.location) {
                        // 合并会话信息和位置数据
                        const sessionInfo = {
                            ...session,
                            location: locationData.location
                        };
                        
                        this.sessionsData.push(sessionInfo);
                        console.log(`会话 ${session.session_id} 位置数据加载成功`);
                    }
                } catch (error) {
                    console.error(`加载会话 ${session.session_id} 位置数据失败:`, error);
                }
            }
            
            // 在地图上显示所有会话
            this.displayAllSessionsOnMap();
            this.updateAllSessionsInfoPanel();
            this.updateStatus(`已加载 ${this.sessionsData.length} 个会话的位置数据`);
            
        } catch (error) {
            console.error('加载所有会话数据失败:', error);
            this.updateStatus('加载会话数据失败: ' + error.message);
        }
    }

    // 重新加载所有会话数据
    async reloadAllSessions() {
        console.log('重新加载所有会话数据');
        this.clearAllSessionOverlays();
        await this.loadAllSessions();
    }

    // 在地图上显示所有会话
    displayAllSessionsOnMap() {
        if (!this.map3d || this.sessionsData.length === 0) return;
        
        console.log('在地图上显示所有会话数据');
        
        // 清除现有覆盖物
        this.clearAllSessionOverlays();
        
        // 计算所有会话的整体边界
        const allBounds = this.calculateOverallBounds();
        
        // 为每个会话创建覆盖物
        this.sessionsData.forEach((sessionInfo, index) => {
            this.createSessionOverlay(sessionInfo, index);
        });
        
        // 调整地图视野以显示所有会话
        if (allBounds) {
            this.map3d.setBounds(allBounds, false, [50, 50, 50, 50]);
            
            // 设置合适的3D视角
            setTimeout(() => {
                this.map3d.setPitch(45);
                this.map3d.setRotation(-20);
            }, 1000);
        }
        
        console.log(`已在地图上显示 ${this.sessionsData.length} 个会话`);
    }

    // 计算所有会话的整体边界
    calculateOverallBounds() {
        if (this.sessionsData.length === 0) return null;
        
        let minLat = Infinity, maxLat = -Infinity;
        let minLon = Infinity, maxLon = -Infinity;
        
        this.sessionsData.forEach(sessionInfo => {
            const location = sessionInfo.location;
            if (location && location.bounds) {
                minLat = Math.min(minLat, location.bounds.southwest.lat);
                maxLat = Math.max(maxLat, location.bounds.northeast.lat);
                minLon = Math.min(minLon, location.bounds.southwest.lon);
                maxLon = Math.max(maxLon, location.bounds.northeast.lon);
            }
        });
        
        if (minLat === Infinity) return null;
        
        return new AMap.Bounds([minLon, minLat], [maxLon, maxLat]);
    }

    // 为单个会话创建覆盖物
    createSessionOverlay(sessionInfo, index) {
        const location = sessionInfo.location;
        if (!location || !location.points || location.points.length < 3) {
            console.warn(`会话 ${sessionInfo.session_id} 缺少有效的位置数据`);
            return;
        }
        
        // 为不同会话使用不同颜色
        const colors = [
            '#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', 
            '#FCEA2B', '#FF9FF3', '#54A0FF', '#5F27CD',
            '#FD79A8', '#FDCB6E', '#6C5CE7', '#A29BFE'
        ];
        const color = colors[index % colors.length];
        
        // 创建多边形覆盖物
        const polygon = new AMap.Polygon({
            path: location.points.map(p => [p.lon, p.lat]),
            strokeColor: color,
            strokeWeight: 3,
            strokeOpacity: 0.8,
            fillColor: color,
            fillOpacity: 0.3,
            zIndex: 50 + index,
            cursor: 'pointer'
        });
        
        // 创建中心标记
        const centerMarker = new AMap.Marker({
            position: [location.center.lon, location.center.lat],
            title: sessionInfo.session_id,
            icon: new AMap.Icon({
                size: new AMap.Size(24, 30),
                image: 'https://webapi.amap.com/theme/v1.3/markers/n/mark_r.png',
                imageSize: new AMap.Size(24, 30)
            }),
            zIndex: 100 + index,
            cursor: 'pointer'
        });
        
        // 绑定鼠标事件
        this.bindSessionEvents(polygon, sessionInfo);
        this.bindSessionEvents(centerMarker, sessionInfo);
        
        // 添加到地图
        this.map3d.add([polygon, centerMarker]);
        
        // 存储覆盖物引用
        this.sessionOverlays.push({
            sessionInfo: sessionInfo,
            polygon: polygon,
            marker: centerMarker,
            color: color
        });
    }

    // 绑定会话覆盖物的鼠标事件
    bindSessionEvents(overlay, sessionInfo) {
        // 鼠标移入事件
        overlay.on('mouseover', (e) => {
            this.showSessionTooltip(e, sessionInfo);
            this.currentHoverSession = sessionInfo.session_id;
        });
        
        // 鼠标移出事件
        overlay.on('mouseout', (e) => {
            this.hideSessionTooltip();
            this.currentHoverSession = null;
        });
        
        // 点击事件 - 跳转到点云可视化
        overlay.on('click', (e) => {
            this.navigateToPointcloud(sessionInfo.session_id);
        });
    }

    // 显示会话信息提示框
    showSessionTooltip(event, sessionInfo) {
        if (!this.infoWindow) {
            this.infoWindow = new AMap.InfoWindow({
                offset: new AMap.Pixel(0, -30),
                closeWhenClickMap: false,
                autoMove: true
            });
        }
        
        const date = new Date(sessionInfo.created_time).toLocaleDateString('zh-CN', {
            year: 'numeric',
            month: '2-digit',
            day: '2-digit',
            hour: '2-digit',
            minute: '2-digit'
        });
        
        const content = `
            <div style="padding: 10px; min-width: 200px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.3);">
                <h4 style="margin: 0 0 8px 0; font-size: 14px; font-weight: bold;">🗂️ ${sessionInfo.session_id}</h4>
                <div style="font-size: 12px; line-height: 1.4;">
                    <p style="margin: 4px 0;"><span style="color: #FFD700;">📅</span> ${date}</p>
                    <p style="margin: 4px 0;"><span style="color: #90EE90;">📷</span> 图片: ${sessionInfo.image_count || 0} 张</p>
                    <p style="margin: 4px 0;"><span style="color: #87CEEB;">🎭</span> 掩码: ${sessionInfo.mask_count || 0} 个</p>
                    <p style="margin: 4px 0;"><span style="color: #FFA500;">📍</span> 检测点: ${sessionInfo.location?.total_points || 0} 个</p>
                    <p style="margin: 4px 0;"><span style="color: #FF69B4;">☁️</span> 点云: ${sessionInfo.has_pointcloud ? '✅ 有' : '❌ 无'}</p>
                </div>
                <div style="text-align: center; margin-top: 8px; font-size: 11px; color: #E0E0E0;">
                    🖱️ 点击查看点云可视化
                </div>
            </div>
        `;
        
        this.infoWindow.setContent(content);
        this.infoWindow.open(this.map3d, event.lnglat);
    }

    // 隐藏会话信息提示框
    hideSessionTooltip() {
        if (this.infoWindow) {
            this.infoWindow.close();
        }
    }

    // 跳转到点云可视化页面
    navigateToPointcloud(sessionId) {
        console.log(`导航到点云可视化页面，会话ID: ${sessionId}`);
        
        // 设置当前会话
        if (window.app) {
            window.app.currentSession = sessionId;
            
            // 更新所有会话选择器
            const selectors = document.querySelectorAll('.session-selector, #session-selector, #review-session-selector, #analysis-session-selector, #stats-session-selector, #map3d-session-selector');
            selectors.forEach(selector => {
                const option = selector.querySelector(`option[value="${sessionId}"]`);
                if (option) {
                    selector.value = sessionId;
                }
            });
            
            // 切换到点云可视化页面
            window.app.switchPage('pointcloud');
            
            // 通知点云可视化页面加载数据 - 增加重试机制
            const loadPointcloudWithRetry = (retryCount = 0) => {
                if (window.pointcloudViewer) {
                    console.log('找到点云可视化器，开始加载数据');
                    window.pointcloudViewer.loadSession(sessionId);
                } else if (retryCount < 5) {
                    console.log(`点云可视化器未找到，${500 * (retryCount + 1)}ms后重试 (${retryCount + 1}/5)`);
                    setTimeout(() => {
                        loadPointcloudWithRetry(retryCount + 1);
                    }, 500 * (retryCount + 1));
                } else {
                    console.error('点云可视化器初始化失败，无法自动加载点云数据');
                }
            };
            
            setTimeout(() => {
                loadPointcloudWithRetry();
            }, 200);
            
            // 显示通知
            if (window.app.showNotification) {
                window.app.showNotification(`已切换到会话 ${sessionId} 的点云可视化`, 'success');
            }
        } else {
            console.error('主应用实例未找到，无法切换页面');
        }
    }

    // 清除所有会话覆盖物
    clearAllSessionOverlays() {
        if (this.sessionOverlays.length > 0) {
            this.sessionOverlays.forEach(overlay => {
                this.map3d.remove([overlay.polygon, overlay.marker]);
            });
            this.sessionOverlays = [];
        }
        
        if (this.infoWindow) {
            this.infoWindow.close();
        }
    }

    // 切换会话覆盖物显示/隐藏
    toggleSessionOverlays() {
        this.sessionOverlays.forEach(overlay => {
            const currentVisible = overlay.polygon.getOptions().fillOpacity > 0;
            if (currentVisible) {
                overlay.polygon.setOptions({ fillOpacity: 0, strokeOpacity: 0 });
                overlay.marker.hide();
            } else {
                overlay.polygon.setOptions({ fillOpacity: 0.3, strokeOpacity: 0.8 });
                overlay.marker.show();
            }
        });
    }

    // 更新状态信息
    updateStatus(message) {
        console.log('状态更新:', message);
        const statusEl = document.getElementById('map3d-status-indicator');
        if (statusEl) {
            statusEl.textContent = message;
        }
    }
    
    // 更新地图位置数据
    updateMapWithLocation(locationData) {
        if (!this.map3d) return;
        
        // 清除之前的普通覆盖物
        this.map3d.clearMap();
        
        if (locationData.bounds) {
            // 创建检测区域边界
            const rectangle = new AMap.Rectangle({
                bounds: new AMap.Bounds(
                    [locationData.bounds.southwest.lon, locationData.bounds.southwest.lat],
                    [locationData.bounds.northeast.lon, locationData.bounds.northeast.lat]
                ),
                strokeColor: '#0066FF',
                strokeWeight: 2,
                strokeOpacity: 0.6,
                fillColor: '#0066FF',
                fillOpacity: 0.05,
                zIndex: 10
            });
            
            this.map3d.add(rectangle);
            
            // 调整地图视野
            this.map3d.setBounds(rectangle.getBounds(), false, [100, 100, 100, 100]);
            
            // 设置3D视角
            setTimeout(() => {
                this.map3d.setPitch(60);
                this.map3d.setRotation(-20);
            }, 1500);
        }
        
        // 添加中心标记
        if (locationData.center) {
            const centerMarker = new AMap.Marker({
                position: [locationData.center.lon, locationData.center.lat],
                title: '建筑检测中心',
                icon: new AMap.Icon({
                    size: new AMap.Size(32, 38),
                    image: 'https://webapi.amap.com/theme/v1.3/markers/n/mark_r.png'
                }),
                zIndex: 150
            });
            
            this.map3d.add(centerMarker);
            
            // 添加信息窗体
            const infoWindow = new AMap.InfoWindow({
                content: `
                    <div style="padding: 15px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; border-radius: 8px;">
                        <h4 style="margin: 0 0 10px 0;">🏗️ 建筑检测中心</h4>
                        <p style="margin: 5px 0; font-size: 13px;">📍 ${locationData.center.lat.toFixed(6)}, ${locationData.center.lon.toFixed(6)}</p>
                        <p style="margin: 5px 0; font-size: 13px;">🔍 检测点数: ${locationData.total_points}</p>
                        <p style="margin: 5px 0; font-size: 13px;">📊 状态: <span style="color: #00ff88;">🟢 实时监测</span></p>
                        <p style="margin: 5px 0; font-size: 13px;">⚡ 特效: <span style="color: #ffff00;">🟡 高亮显示</span></p>
                    </div>
                `,
                offset: new AMap.Pixel(0, -40)
            });
            
            centerMarker.on('click', () => {
                infoWindow.open(this.map3d, centerMarker.getPosition());
            });
        }
    }
    
    // 创建建筑物特效高亮
    createBuildingHighlight(locationData) {
        if (!locationData.points || locationData.points.length < 3) {
            console.warn('需要至少3个点才能创建建筑物高亮');
            return;
        }
        
        // 清除之前的特效
        this.clearBuildingHighlight();
        
        const points = locationData.points.map(point => [point.lon, point.lat]);
        
        // 创建多层特效
        this.createGlowPolygon(points);
        this.createAnimatedBorder(points);
        this.createCornerMarkers(points);
        this.createPulseCenter(locationData.center);
        
        console.log(`建筑物特效高亮已创建，连接 ${points.length} 个点`);
    }
    
    // 创建发光多边形
    createGlowPolygon(points) {
        // 外层发光效果
        const outerGlow = new AMap.Polygon({
            path: points,
            strokeColor: '#00FF88',
            strokeWeight: 4,
            strokeOpacity: 0.8,
            fillColor: '#00FF88',
            fillOpacity: 0.15,
            zIndex: 80
        });
        
        // 内层发光效果
        const innerGlow = new AMap.Polygon({
            path: points,
            strokeColor: '#FFFF00',
            strokeWeight: 2,
            strokeOpacity: 0.9,
            fillColor: '#FFFF00',
            fillOpacity: 0.1,
            zIndex: 85
        });
        
        this.map3d.add([outerGlow, innerGlow]);
        this.buildingHighlights.push(outerGlow, innerGlow);
        
        // 呼吸动画
        let breathOpacity = 0.15;
        let breathDirection = 1;
        const breathInterval = setInterval(() => {
            breathOpacity += 0.01 * breathDirection;
            if (breathOpacity >= 0.25) breathDirection = -1;
            if (breathOpacity <= 0.05) breathDirection = 1;
            
            outerGlow.setOptions({ fillOpacity: breathOpacity });
            innerGlow.setOptions({ fillOpacity: breathOpacity * 0.7 });
        }, 80);
        
        this.pathAnimations.push({ type: 'interval', interval: breathInterval });
    }
    
    // 创建动画边框
    createAnimatedBorder(points) {
        // 主边框线
        const borderLine = new AMap.Polyline({
            path: points,
            strokeColor: '#FF0066',
            strokeWeight: 3,
            strokeOpacity: 0.9,
            strokeStyle: 'solid',
            zIndex: 100
        });
        
        // 动画线条
        const animatedLine = new AMap.Polyline({
            path: points,
            strokeColor: '#FFFFFF',
            strokeWeight: 5,
            strokeOpacity: 0.7,
            strokeStyle: 'dashed',
            zIndex: 95
        });
        
        this.map3d.add([borderLine, animatedLine]);
        this.buildingHighlights.push(borderLine, animatedLine);
        
        // 流动动画
        let dashOffset = 0;
        const flowInterval = setInterval(() => {
            dashOffset += 2;
            if (dashOffset > 20) dashOffset = 0;
            
            animatedLine.setOptions({
                strokeDasharray: [8, 8],
                strokeDashoffset: dashOffset
            });
        }, 100);
        
        this.pathAnimations.push({ type: 'interval', interval: flowInterval });
    }
    
    // 创建角点标记
    createCornerMarkers(points) {
        points.forEach((point, index) => {
            if (index === points.length - 1) return; // 跳过重复的闭合点
            
            // 主标记
            const cornerMarker = new AMap.CircleMarker({
                center: point,
                radius: 6,
                strokeColor: '#FFFFFF',
                strokeWeight: 2,
                fillColor: index === 0 ? '#FF0000' : '#00FF88',
                fillOpacity: 0.9,
                zIndex: 120
            });
            
            // 光晕效果
            const haloMarker = new AMap.CircleMarker({
                center: point,
                radius: 12,
                strokeColor: index === 0 ? '#FF0000' : '#00FF88',
                strokeWeight: 1,
                fillColor: index === 0 ? '#FF0000' : '#00FF88',
                fillOpacity: 0.3,
                zIndex: 115
            });
            
            this.map3d.add([cornerMarker, haloMarker]);
            this.buildingHighlights.push(cornerMarker, haloMarker);
            
            // 脉冲动画
            let pulseRadius = 6;
            let pulseDirection = 1;
            const pulseInterval = setInterval(() => {
                pulseRadius += 0.3 * pulseDirection;
                if (pulseRadius >= 9) pulseDirection = -1;
                if (pulseRadius <= 6) pulseDirection = 1;
                
                cornerMarker.setRadius(pulseRadius);
                haloMarker.setRadius(pulseRadius * 2);
            }, 120);
            
            this.pathAnimations.push({ type: 'interval', interval: pulseInterval });
        });
    }
    
    // 创建中心脉冲效果
    createPulseCenter(center) {
        if (!center) return;
        
        const pulseCenter = new AMap.CircleMarker({
            center: [center.lon, center.lat],
            radius: 20,
            strokeColor: '#FFFF00',
            strokeWeight: 3,
            strokeOpacity: 0.8,
            fillColor: '#FFFF00',
            fillOpacity: 0.2,
            zIndex: 110
        });
        
        this.map3d.add(pulseCenter);
        this.buildingHighlights.push(pulseCenter);
        
        // 扩散动画
        let expandRadius = 20;
        let expandOpacity = 0.2;
        const expandInterval = setInterval(() => {
            expandRadius += 1;
            expandOpacity -= 0.008;
            
            if (expandRadius >= 40) {
                expandRadius = 20;
                expandOpacity = 0.2;
            }
            
            pulseCenter.setRadius(expandRadius);
            pulseCenter.setOptions({ fillOpacity: expandOpacity });
        }, 50);
        
        this.pathAnimations.push({ type: 'interval', interval: expandInterval });
    }
    
    // 清除建筑物高亮
    clearBuildingHighlight() {
        // 清除动画
        this.pathAnimations.forEach(item => {
            if (item.type === 'interval') {
                clearInterval(item.interval);
            }
        });
        this.pathAnimations = [];
        
        // 清除覆盖物
        if (this.buildingHighlights.length > 0) {
            this.map3d.remove(this.buildingHighlights);
            this.buildingHighlights = [];
        }
    }
    
    // 切换建筑物高亮
    toggleBuildingHighlight() {
        if (this.highlightEnabled) {
            this.clearBuildingHighlight();
            this.highlightEnabled = false;
            console.log('建筑物特效已关闭');
        } else {
            this.createBuildingHighlight(this.demoLocationData);
            this.highlightEnabled = true;
            console.log('建筑物特效已开启');
        }
    }
    
    // 重置3D视角
    reset3DView() {
        if (!this.map3d) return;
        
        this.map3d.setZoom(17);
        this.map3d.setPitch(50);
        this.map3d.setRotation(-15);
        this.map3d.setCenter([116.397428, 39.90923]);
    }
    
    // 更新信息面板
    updateInfoPanel(locationData) {
        const pointCountEl = document.getElementById('map3d-point-count');
        if (pointCountEl) {
            pointCountEl.textContent = locationData.total_points || 0;
        }
        
        // 计算覆盖面积（简单估算）
        if (locationData.bounds) {
            const latDiff = locationData.bounds.northeast.lat - locationData.bounds.southwest.lat;
            const lonDiff = locationData.bounds.northeast.lon - locationData.bounds.southwest.lon;
            const area = Math.round(latDiff * lonDiff * 111000 * 111000); // 粗略计算
            
            const coverageAreaEl = document.getElementById('map3d-coverage-area');
            if (coverageAreaEl) {
                coverageAreaEl.textContent = area.toLocaleString() + ' m²';
            }
        }
    }

    // 更新所有会话的信息面板
    updateAllSessionsInfoPanel() {
        const sessionCountEl = document.getElementById('map3d-session-count');
        const pointCountEl = document.getElementById('map3d-point-count');
        const buildingCountEl = document.getElementById('map3d-building-count');
        
        if (sessionCountEl) {
            sessionCountEl.textContent = this.sessionsData.length;
        }
        
        // 计算总检测点数
        const totalPoints = this.sessionsData.reduce((sum, session) => {
            return sum + (session.location?.total_points || 0);
        }, 0);
        
        if (pointCountEl) {
            pointCountEl.textContent = totalPoints;
        }
        
        if (buildingCountEl) {
            buildingCountEl.textContent = this.sessionsData.length;
        }
        
        // 更新状态
        this.updateStatus(`已显示 ${this.sessionsData.length} 个会话，共 ${totalPoints} 个检测点`);
    }
    
    // 更新视角信息
    updateViewInfo() {
        if (!this.map3d) return;
        
        const pitch = Math.round(this.map3d.getPitch());
        const viewHeightEl = document.getElementById('map3d-view-height');
        if (viewHeightEl) {
            viewHeightEl.textContent = pitch + '°';
        }
    }
}

// 页面加载完成后自动初始化
document.addEventListener('DOMContentLoaded', () => {
    console.log('3D建筑物检测地图管理器已加载');
    
    // 立即创建Map3DManager实例
    if (!window.map3DManager) {
        console.log('创建全局Map3DManager实例');
        window.map3DManager = new Map3DManager();
    }
});

// 全局函数，供页面切换时调用
window.initMap3DWhenVisible = function() {
    console.log('页面切换到3D地图，尝试初始化地图');
    if (window.map3DManager) {
        window.map3DManager.initMap3D();
    } else {
        console.log('Map3DManager不存在，创建新实例');
        window.map3DManager = new Map3DManager();
    }
};

// 全局测试函数，可以直接在控制台调用
window.testMap3D = function() {
    console.log('测试3D地图功能');
    if (window.map3DManager) {
        console.log('Map3DManager存在，尝试加载演示数据');
        window.map3DManager.loadDemoData();
    } else {
        console.log('Map3DManager不存在');
    }
};
