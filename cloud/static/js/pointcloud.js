// 点云可视化管理器
// 定义OrbitControls类
class OrbitControls {
    constructor(camera, domElement) {
        this.camera = camera;
        this.domElement = domElement;
        
        this.enableDamping = true;
        this.dampingFactor = 0.05;
        this.enableZoom = true;
        this.enableRotate = true;
        this.enablePan = true;
        
        this.minDistance = 0;
        this.maxDistance = Infinity;
        
        this.target = new THREE.Vector3();
        this.position0 = this.camera.position.clone();
        this.target0 = this.target.clone();
        
        this.spherical = new THREE.Spherical();
        this.sphericalDelta = new THREE.Spherical();
        
        this.scale = 1;
        this.panOffset = new THREE.Vector3();
        
        this.rotateStart = new THREE.Vector2();
        this.rotateEnd = new THREE.Vector2();
        this.rotateDelta = new THREE.Vector2();
        
        this.panStart = new THREE.Vector2();
        this.panEnd = new THREE.Vector2();
        this.panDelta = new THREE.Vector2();
        
        this.dollyStart = new THREE.Vector2();
        this.dollyEnd = new THREE.Vector2();
        this.dollyDelta = new THREE.Vector2();
        
        this.state = 'NONE';
        this.EPS = 0.000001;
        
        this.addEventListeners();
    }
    
    addEventListeners() {
        this.domElement.addEventListener('mousedown', this.onMouseDown.bind(this));
        this.domElement.addEventListener('wheel', this.onMouseWheel.bind(this));
        this.domElement.addEventListener('contextmenu', this.onContextMenu.bind(this));
        
        window.addEventListener('mousemove', this.onMouseMove.bind(this));
        window.addEventListener('mouseup', this.onMouseUp.bind(this));
    }
    
    onMouseDown(event) {
        event.preventDefault();
        
        if (event.button === 0) {
            this.state = 'ROTATE';
            this.rotateStart.set(event.clientX, event.clientY);
        } else if (event.button === 2) {
            this.state = 'PAN';
            this.panStart.set(event.clientX, event.clientY);
        }
    }
    
    onMouseMove(event) {
        if (this.state === 'ROTATE') {
            this.rotateEnd.set(event.clientX, event.clientY);
            this.rotateDelta.subVectors(this.rotateEnd, this.rotateStart);
            
            // 降低旋转敏感度，修复旋转方向
            this.rotateLeft(2 * Math.PI * this.rotateDelta.x / this.domElement.clientWidth * 0.3);
            this.rotateUp(2 * Math.PI * this.rotateDelta.y / this.domElement.clientHeight * 0.3);
            
            this.rotateStart.copy(this.rotateEnd);
            this.update();
        } else if (this.state === 'PAN') {
            this.panEnd.set(event.clientX, event.clientY);
            this.panDelta.subVectors(this.panEnd, this.panStart);
            
            this.pan(this.panDelta.x, this.panDelta.y);
            
            this.panStart.copy(this.panEnd);
            this.update();
        }
    }
    
    onMouseUp() {
        this.state = 'NONE';
    }
    
    onMouseWheel(event) {
        event.preventDefault();
        
        // 修复缩放方向，降低缩放敏感度
        if (event.deltaY < 0) {
            this.dollyOut(); // 向上滚动放大
        } else {
            this.dollyIn(); // 向下滚动缩小
        }
        
        this.update();
    }
    
    onContextMenu(event) {
        event.preventDefault();
    }
    
    rotateLeft(angle) {
        this.sphericalDelta.theta -= angle;
    }
    
    rotateUp(angle) {
        this.sphericalDelta.phi -= angle;
    }
    
    pan(deltaX, deltaY) {
        const element = this.domElement;
        
        if (this.camera.isPerspectiveCamera) {
            const position = this.camera.position;
            const offset = position.clone().sub(this.target);
            let targetDistance = offset.length();
            
            targetDistance *= Math.tan((this.camera.fov / 2) * Math.PI / 180.0);
            
            this.panLeft(2 * deltaX * targetDistance / element.clientHeight, this.camera.matrix);
            this.panUp(2 * deltaY * targetDistance / element.clientHeight, this.camera.matrix);
        }
    }
    
    panLeft(distance, objectMatrix) {
        const v = new THREE.Vector3();
        v.setFromMatrixColumn(objectMatrix, 0);
        v.multiplyScalar(-distance);
        this.panOffset.add(v);
    }
    
    panUp(distance, objectMatrix) {
        const v = new THREE.Vector3();
        v.setFromMatrixColumn(objectMatrix, 1);
        v.multiplyScalar(distance);
        this.panOffset.add(v);
    }
    
    dollyIn() {
        this.scale /= 0.95; // 缩小 (向内推进)
    }
    
    dollyOut() {
        this.scale *= 0.95; // 放大 (向外拉远)
    }
    
    update() {
        const offset = new THREE.Vector3();
        const quat = new THREE.Quaternion().setFromUnitVectors(this.camera.up, new THREE.Vector3(0, 1, 0));
        const quatInverse = quat.clone().invert();
        
        const position = this.camera.position;
        
        offset.copy(position).sub(this.target);
        offset.applyQuaternion(quat);
        
        this.spherical.setFromVector3(offset);
        this.spherical.theta += this.sphericalDelta.theta;
        this.spherical.phi += this.sphericalDelta.phi;
        
        this.spherical.radius *= this.scale;
        
        this.spherical.makeSafe();
        
        offset.setFromSpherical(this.spherical);
        offset.applyQuaternion(quatInverse);
        
        position.copy(this.target).add(offset);
        
        this.camera.lookAt(this.target);
        
        if (this.enableDamping) {
            this.sphericalDelta.theta *= (1 - this.dampingFactor);
            this.sphericalDelta.phi *= (1 - this.dampingFactor);
        } else {
            this.sphericalDelta.set(0, 0, 0);
        }
        
        this.scale = 1;
        this.panOffset.set(0, 0, 0);
        
        this.target.add(this.panOffset);
        
        return true;
    }
    
    reset() {
        this.target.copy(this.target0);
        this.camera.position.copy(this.position0);
        this.camera.zoom = 1;
        this.camera.updateProjectionMatrix();
        this.update();
    }
}

class PointcloudViewer {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.pointCloud = null;
        this.currentSession = null;
        this.mapInstance = null;
        
        // 渲染质量参数
        this.renderParams = {
            pointSize: 0.02,
            pointDensity: 5, // 1-10, 对应10%-100%的点
            renderDistance: 5000,
            qualityLevel: 2 // 1:低, 2:中, 3:高, 4:超高
        };
        
        // 原始点云数据缓存
        this.originalPointcloudData = null;
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
        this.initMap();
    }
    
    setupEventListeners() {
        // 加载点云按钮
        const loadBtn = document.getElementById('load-pointcloud-btn');
        if (loadBtn) {
            loadBtn.addEventListener('click', () => {
                this.loadPointcloud();
            });
        }
        
        // 重置视角按钮
        const resetBtn = document.getElementById('reset-view-btn');
        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                this.resetView();
            });
        }
        
        // 切换颜色按钮
        const colorBtn = document.getElementById('toggle-colors-btn');
        if (colorBtn) {
            colorBtn.addEventListener('click', () => {
                this.toggleColors();
            });
        }
        
        // 会话选择器
        const sessionSelector = document.getElementById('session-selector');
        if (sessionSelector) {
            sessionSelector.addEventListener('change', (e) => {
                this.currentSession = e.target.value;
            });
        }
        
        // 渲染质量控制滑动条
        this.setupQualityControls();
    }
    
    setupQualityControls() {
        // 点大小滑动条
        const pointSizeSlider = document.getElementById('point-size-slider');
        const pointSizeValue = document.getElementById('point-size-value');
        if (pointSizeSlider && pointSizeValue) {
            pointSizeSlider.addEventListener('input', (e) => {
                this.renderParams.pointSize = parseFloat(e.target.value);
                pointSizeValue.textContent = this.renderParams.pointSize.toFixed(3);
                this.updatePointSize();
            });
        }
        
        // 密度控制滑动条
        const densitySlider = document.getElementById('point-density-slider');
        const densityValue = document.getElementById('point-density-value');
        if (densitySlider && densityValue) {
            densitySlider.addEventListener('input', (e) => {
                this.renderParams.pointDensity = parseInt(e.target.value);
                densityValue.textContent = (this.renderParams.pointDensity * 10) + '%';
                this.updatePointDensity();
            });
        }
        
        // 渲染距离滑动条
        const distanceSlider = document.getElementById('render-distance-slider');
        const distanceValue = document.getElementById('render-distance-value');
        if (distanceSlider && distanceValue) {
            distanceSlider.addEventListener('input', (e) => {
                this.renderParams.renderDistance = parseInt(e.target.value);
                distanceValue.textContent = this.renderParams.renderDistance;
                this.updateRenderDistance();
            });
        }
        
        // 质量等级滑动条
        const qualitySlider = document.getElementById('quality-level-slider');
        const qualityValue = document.getElementById('quality-level-value');
        if (qualitySlider && qualityValue) {
            qualitySlider.addEventListener('input', (e) => {
                this.renderParams.qualityLevel = parseInt(e.target.value);
                const levels = ['', '低等', '中等', '高等', '超高'];
                qualityValue.textContent = levels[this.renderParams.qualityLevel];
                this.updateQualityLevel();
            });
        }
        
        // 预设按钮
        const presetButtons = {
            'quality-preset-low': () => this.applyQualityPreset('low'),
            'quality-preset-medium': () => this.applyQualityPreset('medium'),
            'quality-preset-high': () => this.applyQualityPreset('high'),
            'quality-preset-ultra': () => this.applyQualityPreset('ultra')
        };
        
        Object.entries(presetButtons).forEach(([id, handler]) => {
            const btn = document.getElementById(id);
            if (btn) {
                btn.addEventListener('click', handler);
            }
        });
    }

    // 加载指定会话的点云数据
    loadSession(sessionId) {
        console.log(`点云可视化器收到加载会话请求: ${sessionId}`);
        
        // 设置当前会话
        this.currentSession = sessionId;
        
        // 更新会话选择器
        const sessionSelector = document.getElementById('session-selector');
        if (sessionSelector) {
            sessionSelector.value = sessionId;
        }
        
        // 自动加载点云
        setTimeout(() => {
            this.loadPointcloud();
        }, 100);
    }
    
    async loadPointcloud() {
        if (!this.currentSession) {
            window.app.showNotification('请先选择一个会话', 'error');
            return;
        }
        
        try {
            const loadBtn = document.getElementById('load-pointcloud-btn');
            loadBtn.disabled = true;
            loadBtn.textContent = '加载中...';
            
            // 获取点云数据
            const response = await fetch(`/api/pointcloud/${this.currentSession}`);
            const data = await response.json();
            
            if (response.ok) {
                this.renderPointcloud(data.pointcloud);
                this.updatePointcloudInfo(data.pointcloud);
                
                // 加载位置信息
                await this.loadLocationData();
                
                window.app.showNotification('点云加载成功', 'success');
            } else {
                window.app.showNotification(`加载失败: ${data.detail}`, 'error');
            }
            
        } catch (error) {
            console.error('加载点云失败:', error);
            window.app.showNotification(`加载错误: ${error.message}`, 'error');
        } finally {
            const loadBtn = document.getElementById('load-pointcloud-btn');
            loadBtn.disabled = false;
            loadBtn.textContent = '加载点云';
        }
    }
    
    renderPointcloud(pointcloudData) {
        const container = document.getElementById('pointcloud-canvas');
        if (!container) return;
        
        // 保存原始数据
        this.originalPointcloudData = pointcloudData;
        
        // 清理现有内容
        container.innerHTML = '';
        
        // 初始化Three.js
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x000000);
        
        // 设置相机
        this.camera = new THREE.PerspectiveCamera(
            75,
            container.clientWidth / container.clientHeight,
            0.1,
            this.renderParams.renderDistance
        );
        
        // 设置渲染器
        this.renderer = new THREE.WebGLRenderer({ antialias: this.renderParams.qualityLevel >= 2 });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        container.appendChild(this.renderer.domElement);
        
        // 设置控制器
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        
        // 创建点云（应用密度设置）
        const densityRatio = this.renderParams.pointDensity / 10;
        const sampledData = this.samplePointcloud(pointcloudData, Math.floor(pointcloudData.points.length * densityRatio));
        this.createPointCloud(sampledData);
        
        // 设置初始视角
        this.setInitialView(pointcloudData);
        
        // 添加光源
        this.addLights();
        
        // 应用质量设置
        this.updateQualityLevel();
        
        // 开始渲染循环
        this.animate();
        
        // 更新点云信息
        this.updatePointcloudInfo(sampledData);
        
        // 处理窗口大小变化
        window.addEventListener('resize', () => {
            this.handleResize();
        });
    }
    
    createPointCloud(data) {
        const geometry = new THREE.BufferGeometry();
        
        // 设置顶点位置
        const positions = new Float32Array(data.points.flat());
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        
        // 设置颜色
        if (data.colors) {
            const colors = new Float32Array(data.colors.flat());
            geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        } else {
            // 如果没有颜色，根据高度着色
            const colors = this.generateHeightColors(data.points);
            geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        }
        
        // 创建材质
        const material = new THREE.PointsMaterial({
            size: this.renderParams.pointSize,
            vertexColors: true,
            transparent: true,
            opacity: 0.8,
            sizeAttenuation: true
        });
        
        // 创建点云对象
        this.pointCloud = new THREE.Points(geometry, material);
        this.scene.add(this.pointCloud);
    }
    
    generateHeightColors(points) {
        const colors = [];
        const minZ = Math.min(...points.map(p => p[2]));
        const maxZ = Math.max(...points.map(p => p[2]));
        const range = maxZ - minZ;
        
        for (let i = 0; i < points.length; i++) {
            const z = points[i][2];
            const normalized = (z - minZ) / range;
            
            // 使用彩虹色映射
            const hue = (1 - normalized) * 0.7; // 从红色到蓝色
            const color = new THREE.Color().setHSL(hue, 1, 0.5);
            
            colors.push(color.r, color.g, color.b);
        }
        
        return new Float32Array(colors);
    }
    
    setInitialView(data) {
        if (!data.bounds) return;
        
        const center = [
            (data.bounds.min[0] + data.bounds.max[0]) / 2,
            (data.bounds.min[1] + data.bounds.max[1]) / 2,
            (data.bounds.min[2] + data.bounds.max[2]) / 2
        ];
        
        const size = Math.max(
            data.bounds.max[0] - data.bounds.min[0],
            data.bounds.max[1] - data.bounds.min[1],
            data.bounds.max[2] - data.bounds.min[2]
        );
        
        this.camera.position.set(
            center[0] + size,
            center[1] + size,
            center[2] + size
        );
        
        this.controls.target.set(center[0], center[1], center[2]);
        this.controls.update();
    }
    
    addLights() {
        // 环境光
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        this.scene.add(ambientLight);
        
        // 方向光
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(100, 100, 100);
        this.scene.add(directionalLight);
    }
    
    animate() {
        if (!this.renderer) return;
        
        requestAnimationFrame(() => this.animate());
        
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
    
    handleResize() {
        const container = document.getElementById('pointcloud-canvas');
        if (!container || !this.camera || !this.renderer) return;
        
        const width = container.clientWidth;
        const height = container.clientHeight;
        
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        
        this.renderer.setSize(width, height);
    }
    
    resetView() {
        if (!this.controls) return;
        
        this.controls.reset();
        window.app.showNotification('视角已重置', 'info');
    }
    
    toggleColors() {
        if (!this.pointCloud) return;
        
        const material = this.pointCloud.material;
        material.vertexColors = !material.vertexColors;
        material.needsUpdate = true;
        
        const status = material.vertexColors ? '彩色' : '单色';
        window.app.showNotification(`已切换到${status}模式`, 'info');
    }
    
    updatePointcloudInfo(data) {
        if (!data.stats) return;
        
        // 更新点云统计信息
        document.getElementById('points-count').textContent = data.stats.total_points.toLocaleString();
        
        if (data.bounds) {
            const bounds = data.bounds;
            document.getElementById('bounding-box').textContent = 
                `${bounds.min[0].toFixed(2)}, ${bounds.min[1].toFixed(2)}, ${bounds.min[2].toFixed(2)} ~ ${bounds.max[0].toFixed(2)}, ${bounds.max[1].toFixed(2)}, ${bounds.max[2].toFixed(2)}`;
        }
        
        if (data.stats.center) {
            const center = data.stats.center;
            document.getElementById('center-point').textContent = 
                `${center[0].toFixed(2)}, ${center[1].toFixed(2)}, ${center[2].toFixed(2)}`;
        }
    }
    
    initMap() {
        // 等待高德地图API加载完成
        const initAMap = () => {
            if (window.AMap) {
                try {
                    this.mapInstance = new AMap.Map('amap-container', {
                        zoom: 15,
                        center: [116.397428, 39.90923], // 默认中心点
                        mapStyle: 'amap://styles/dark'
                    });
                    console.log('高德地图初始化成功');
                } catch (error) {
                    console.error('地图初始化失败:', error);
                }
            } else {
                console.log('等待高德地图API加载...');
                // 监听高德地图加载完成事件
                window.addEventListener('amapLoaded', (e) => {
                    try {
                        this.mapInstance = new e.detail.Map('amap-container', {
                            zoom: 15,
                            center: [116.397428, 39.90923], // 默认中心点
                            mapStyle: 'amap://styles/dark'
                        });
                        console.log('高德地图初始化成功');
                    } catch (error) {
                        console.error('地图初始化失败:', error);
                    }
                });
            }
        };
        
        // 如果API已加载，直接初始化，否则等待
        initAMap();
    }
    
    async loadLocationData() {
        if (!this.currentSession) return;
        
        try {
            const response = await fetch(`/api/location/${this.currentSession}`);
            const data = await response.json();
            
            if (response.ok && data.location) {
                this.updateMap(data.location);
            }
        } catch (error) {
            console.error('加载位置数据失败:', error);
        }
    }
    
    updateMap(locationData) {
        if (!this.mapInstance) return;
        
        // 清除现有标记
        this.mapInstance.clearMap();
        
        // 设置地图中心
        const center = [locationData.center.lon, locationData.center.lat];
        this.mapInstance.setCenter(center);
        
        // 添加起点标记
        const startMarker = new AMap.Marker({
            position: [locationData.start_point.lon, locationData.start_point.lat],
            title: '起点'
        });
        startMarker.setMap(this.mapInstance);
        
        // 添加终点标记
        const endMarker = new AMap.Marker({
            position: [locationData.end_point.lon, locationData.end_point.lat],
            title: '终点'
        });
        endMarker.setMap(this.mapInstance);
        
        // 添加矩形框
        const rectangle = new AMap.Rectangle({
            bounds: new AMap.Bounds(
                [locationData.start_point.lon, locationData.start_point.lat],
                [locationData.end_point.lon, locationData.end_point.lat]
            ),
            strokeColor: '#00d4ff',
            strokeWeight: 2,
            strokeOpacity: 0.8,
            fillColor: '#00d4ff',
            fillOpacity: 0.2
        });
        
        this.mapInstance.add(rectangle);
        
        // 调整视野
        this.mapInstance.setFitView([rectangle]);
    }
    
    // 渲染质量控制方法
    updatePointSize() {
        if (this.pointCloud && this.pointCloud.material) {
            this.pointCloud.material.size = this.renderParams.pointSize;
            this.pointCloud.material.needsUpdate = true;
        }
    }
    
    updatePointDensity() {
        if (!this.originalPointcloudData) return;
        
        const densityRatio = this.renderParams.pointDensity / 10;
        const totalPoints = this.originalPointcloudData.points.length;
        const targetPoints = Math.floor(totalPoints * densityRatio);
        
        // 对点进行随机采样
        const sampledData = this.samplePointcloud(this.originalPointcloudData, targetPoints);
        
        // 重新创建点云
        if (this.pointCloud) {
            this.scene.remove(this.pointCloud);
        }
        this.createPointCloud(sampledData);
        
        // 更新点云信息
        this.updatePointcloudInfo(sampledData);
    }
    
    updateRenderDistance() {
        if (this.camera) {
            this.camera.far = this.renderParams.renderDistance;
            this.camera.updateProjectionMatrix();
        }
    }
    
    updateQualityLevel() {
        if (!this.renderer) return;
        
        const qualitySettings = {
            1: { // 低质量
                antialias: false,
                pixelRatio: 0.5,
                shadowMapEnabled: false
            },
            2: { // 中等质量
                antialias: true,
                pixelRatio: 1,
                shadowMapEnabled: false
            },
            3: { // 高质量
                antialias: true,
                pixelRatio: window.devicePixelRatio || 1,
                shadowMapEnabled: true
            },
            4: { // 超高质量
                antialias: true,
                pixelRatio: Math.min(window.devicePixelRatio * 1.5, 3) || 2,
                shadowMapEnabled: true
            }
        };
        
        const settings = qualitySettings[this.renderParams.qualityLevel];
        if (settings) {
            this.renderer.setPixelRatio(settings.pixelRatio);
            this.renderer.shadowMap.enabled = settings.shadowMapEnabled;
            if (settings.shadowMapEnabled) {
                this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
            }
        }
    }
    
    samplePointcloud(data, targetCount) {
        const totalPoints = data.points.length;
        if (targetCount >= totalPoints) return data;
        
        // 使用系统采样确保均匀分布
        const step = totalPoints / targetCount;
        const sampledPoints = [];
        const sampledColors = [];
        
        for (let i = 0; i < targetCount; i++) {
            const index = Math.floor(i * step);
            sampledPoints.push(data.points[index]);
            if (data.colors && data.colors[index]) {
                sampledColors.push(data.colors[index]);
            }
        }
        
        return {
            ...data,
            points: sampledPoints,
            colors: sampledColors.length > 0 ? sampledColors : null
        };
    }
    
    applyQualityPreset(preset) {
        const presets = {
            low: {
                pointSize: 0.01,
                pointDensity: 3,
                renderDistance: 2000,
                qualityLevel: 1
            },
            medium: {
                pointSize: 0.02,
                pointDensity: 5,
                renderDistance: 5000,
                qualityLevel: 2
            },
            high: {
                pointSize: 0.04,
                pointDensity: 7,
                renderDistance: 8000,
                qualityLevel: 3
            },
            ultra: {
                pointSize: 0.06,
                pointDensity: 10,
                renderDistance: 10000,
                qualityLevel: 4
            }
        };
        
        const settings = presets[preset];
        if (!settings) return;
        
        // 更新参数
        this.renderParams = { ...settings };
        
        // 更新UI
        this.updateQualityUI();
        
        // 应用设置
        this.updatePointSize();
        this.updatePointDensity();
        this.updateRenderDistance();
        this.updateQualityLevel();
        
        // 更新预设按钮状态
        this.updatePresetButtonStates(preset);
    }
    
    updateQualityUI() {
        // 更新滑动条值
        const pointSizeSlider = document.getElementById('point-size-slider');
        const pointSizeValue = document.getElementById('point-size-value');
        if (pointSizeSlider && pointSizeValue) {
            pointSizeSlider.value = this.renderParams.pointSize;
            pointSizeValue.textContent = this.renderParams.pointSize.toFixed(3);
        }
        
        const densitySlider = document.getElementById('point-density-slider');
        const densityValue = document.getElementById('point-density-value');
        if (densitySlider && densityValue) {
            densitySlider.value = this.renderParams.pointDensity;
            densityValue.textContent = (this.renderParams.pointDensity * 10) + '%';
        }
        
        const distanceSlider = document.getElementById('render-distance-slider');
        const distanceValue = document.getElementById('render-distance-value');
        if (distanceSlider && distanceValue) {
            distanceSlider.value = this.renderParams.renderDistance;
            distanceValue.textContent = this.renderParams.renderDistance;
        }
        
        const qualitySlider = document.getElementById('quality-level-slider');
        const qualityValue = document.getElementById('quality-level-value');
        if (qualitySlider && qualityValue) {
            qualitySlider.value = this.renderParams.qualityLevel;
            const levels = ['', '低等', '中等', '高等', '超高'];
            qualityValue.textContent = levels[this.renderParams.qualityLevel];
        }
    }
    
    updatePresetButtonStates(activePreset) {
        const buttons = ['low', 'medium', 'high', 'ultra'];
        buttons.forEach(preset => {
            const btn = document.getElementById(`quality-preset-${preset}`);
            if (btn) {
                if (preset === activePreset) {
                    btn.classList.add('active');
                } else {
                    btn.classList.remove('active');
                }
            }
        });
    }
    
    dispose() {
        if (this.renderer) {
            this.renderer.dispose();
        }
        if (this.controls) {
            this.controls.dispose();
        }
        if (this.mapInstance) {
            this.mapInstance.destroy();
        }
    }
}

// 全局变量
window.PointcloudViewer = PointcloudViewer;
