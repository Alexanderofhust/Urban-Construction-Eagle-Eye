// 复检管理器
class ReviewManager {
    constructor() {
        this.currentSession = null;
        this.defects = [];
        this.currentDefectIndex = 0;
        this.reviewResults = {};
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
    }
    
    setupEventListeners() {
        // 加载缺陷按钮
        const loadBtn = document.getElementById('load-defects-btn');
        if (loadBtn) {
            loadBtn.addEventListener('click', () => {
                this.loadDefects();
            });
        }
        
        // 会话选择器
        const sessionSelector = document.getElementById('review-session-selector');
        if (sessionSelector) {
            sessionSelector.addEventListener('change', (e) => {
                this.currentSession = e.target.value;
            });
        }
        
        // 复检按钮
        const approveBtn = document.getElementById('approve-btn');
        const rejectBtn = document.getElementById('reject-btn');
        
        if (approveBtn) {
            approveBtn.addEventListener('click', () => {
                this.reviewDefect(true);
            });
        }
        
        if (rejectBtn) {
            rejectBtn.addEventListener('click', () => {
                this.reviewDefect(false);
            });
        }
    }
    
    async loadDefects() {
        if (!this.currentSession) {
            window.app.showNotification('请先选择一个会话', 'error');
            return;
        }
        
        try {
            const loadBtn = document.getElementById('load-defects-btn');
            loadBtn.disabled = true;
            loadBtn.textContent = '加载中...';
            
            const response = await fetch(`/api/defects/${this.currentSession}`);
            const data = await response.json();
            
            if (response.ok) {
                this.defects = data.defects;
                this.renderDefectsList();
                
                if (this.defects.length > 0) {
                    this.showDefect(0);
                } else {
                    window.app.showNotification('该会话没有发现缺陷', 'info');
                }
            } else {
                window.app.showNotification(`加载失败: ${data.detail}`, 'error');
            }
            
        } catch (error) {
            console.error('加载缺陷失败:', error);
            window.app.showNotification(`加载错误: ${error.message}`, 'error');
        } finally {
            const loadBtn = document.getElementById('load-defects-btn');
            loadBtn.disabled = false;
            loadBtn.textContent = '加载缺陷';
        }
    }
    
    renderDefectsList() {
        const defectsList = document.getElementById('defects-list');
        
        if (this.defects.length === 0) {
            defectsList.innerHTML = '<p style="text-align: center; color: rgba(255,255,255,0.7); padding: 2rem;">没有发现缺陷</p>';
            return;
        }
        
        defectsList.innerHTML = this.defects.map((defect, index) => {
            const severityClass = this.getSeverityClass(defect.severity_score);
            const severityText = this.getSeverityText(defect.severity_score);
            
            return `
                <div class="defect-item ${index === this.currentDefectIndex ? 'active' : ''}" 
                     data-index="${index}">
                    <div class="defect-header">
                        <div class="defect-id">${defect.defect_id}</div>
                        <div class="severity-badge ${severityClass}">${severityText}</div>
                    </div>
                    <div class="defect-info">
                        <span>📊 ${defect.severity_score}%</span>
                        <span>📐 ${defect.defect_percentage.toFixed(2)}%</span>
                        <span>🎯 ${Object.keys(defect.defect_types).join(', ')}</span>
                    </div>
                </div>
            `;
        }).join('');
        
        // 添加点击事件
        defectsList.querySelectorAll('.defect-item').forEach(item => {
            item.addEventListener('click', (e) => {
                const index = parseInt(e.currentTarget.dataset.index);
                this.showDefect(index);
            });
        });
    }
    
    getSeverityClass(score) {
        if (score >= 80) return 'severity-high';
        if (score >= 50) return 'severity-medium';
        return 'severity-low';
    }
    
    getSeverityText(score) {
        if (score >= 80) return '严重';
        if (score >= 50) return '中等';
        return '轻微';
    }
    
    showDefect(index) {
        if (index < 0 || index >= this.defects.length) return;
        
        this.currentDefectIndex = index;
        const defect = this.defects[index];
        
        // 更新缺陷列表选中状态
        document.querySelectorAll('.defect-item').forEach(item => {
            item.classList.remove('active');
        });
        document.querySelector(`[data-index="${index}"]`).classList.add('active');
        
        // 显示复检面板
        const reviewPanel = document.getElementById('review-panel');
        reviewPanel.style.display = 'block';
        
        // 更新缺陷信息
        document.getElementById('current-severity').textContent = `${defect.severity_score}% (${this.getSeverityText(defect.severity_score)})`;
        
        // 更新进度
        document.getElementById('review-progress-text').textContent = `${index + 1}/${this.defects.length}`;
        
        // 加载图像
        this.loadDefectImages(defect);
    }
    
    async loadDefectImages(defect) {
        try {
            // 加载原始图像
            const originalImage = document.getElementById('original-image');
            originalImage.src = `/api/image/${this.currentSession}/images/${defect.image_file}`;
            
            // 创建mask叠加显示
            await this.createMaskOverlay(defect);
            
            // 图像加载错误处理
            originalImage.onerror = () => {
                originalImage.src = 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMzAwIiBoZWlnaHQ9IjIwMCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48cmVjdCB3aWR0aD0iMzAwIiBoZWlnaHQ9IjIwMCIgZmlsbD0iIzMzMzMzMyIvPjx0ZXh0IHg9IjE1MCIgeT0iMTAwIiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSIjNjY2NjY2IiBmb250LWZhbWlseT0iQXJpYWwiIGZvbnQtc2l6ZT0iMTQiPuWbvuWDj+WKoOi9veWksei0pTwvdGV4dD48L3N2Zz4=';
            };
            
        } catch (error) {
            console.error('加载图像失败:', error);
        }
    }
    
    async createMaskOverlay(defect) {
        return new Promise((resolve, reject) => {
            const originalImage = document.getElementById('original-image');
            const overlayImage = document.getElementById('overlay-image');
            
            // 加载原始图像
            const originalImg = new Image();
            originalImg.crossOrigin = 'anonymous';
            originalImg.onload = () => {
                // 加载mask图像
                const maskImg = new Image();
                maskImg.crossOrigin = 'anonymous';
                maskImg.onload = () => {
                    // 创建canvas进行叠加
                    const canvas = document.createElement('canvas');
                    const ctx = canvas.getContext('2d');
                    
                    canvas.width = originalImg.width;
                    canvas.height = originalImg.height;
                    
                    // 绘制原始图像
                    ctx.drawImage(originalImg, 0, 0);
                    
                    // 创建mask叠加
                    this.overlayMask(ctx, maskImg, canvas.width, canvas.height);
                    
                    // 设置叠加图像
                    overlayImage.src = canvas.toDataURL();
                    resolve();
                };
                
                maskImg.onerror = () => {
                    overlayImage.src = 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMzAwIiBoZWlnaHQ9IjIwMCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48cmVjdCB3aWR0aD0iMzAwIiBoZWlnaHQ9IjIwMCIgZmlsbD0iIzMzMzMzMyIvPjx0ZXh0IHg9IjE1MCIgeT0iMTAwIiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSIjNjY2NjY2IiBmb250LWZhbWlseT0iQXJpYWwiIGZvbnQtc2l6ZT0iMTQiPk1hc2vmoYoFDXR9sVSwT+WqguWImeaAnyStEzwvdGV4dD48L3N2Zz4=';
                    resolve();
                };
                
                maskImg.src = `/api/image/${this.currentSession}/masks/${defect.mask_file}`;
            };
            
            originalImg.onerror = () => {
                reject(new Error('原始图像加载失败'));
            };
            
            originalImg.src = `/api/image/${this.currentSession}/images/${defect.image_file}`;
        });
    }
    
    overlayMask(ctx, maskImg, width, height) {
        // 创建临时canvas来处理mask
        const tempCanvas = document.createElement('canvas');
        const tempCtx = tempCanvas.getContext('2d');
        
        tempCanvas.width = width;
        tempCanvas.height = height;
        
        // 绘制mask图像到临时canvas
        tempCtx.drawImage(maskImg, 0, 0, width, height);
        
        // 获取mask像素数据
        const maskData = tempCtx.getImageData(0, 0, width, height);
        const maskPixels = maskData.data;
        
        // 获取原始图像数据
        const originalData = ctx.getImageData(0, 0, width, height);
        const originalPixels = originalData.data;
        
        // 参考predict_new.py中的类别颜色映射
        const class_colors = [
            [0, 0, 0],        // 类别0: 黑色 (背景)
            [255, 0, 0],      // 类别1: 红色
            [0, 255, 0],      // 类别2: 绿色
            [0, 0, 255],      // 类别3: 蓝色
            [255, 255, 0],    // 类别4: 黄色
            [255, 0, 255],    // 类别5: 品红
            [0, 255, 255],    // 类别6: 青色
        ];
        
        // 处理每个像素，进行颜色混合 (参考predict_new.py的可视化方法)
        for (let i = 0; i < maskPixels.length; i += 4) {
            // 获取mask像素值 (使用红色通道作为类别ID)
            const maskR = maskPixels[i];
            const maskG = maskPixels[i + 1]; 
            const maskB = maskPixels[i + 2];
            
            // 将灰度值转换为类别ID (与predict_new.py中的方法一致)
            const gray = (maskR + maskG + maskB) / 3;
            let classId = 0;
            
            // 根据灰度值确定类别 (假设mask是灰度图，每个灰度级对应一个类别)
            if (gray >= 3) {
                classId = 3; // 最亮的区域
            } else if (gray == 2) {
                classId = 2;
            } else if (gray == 1) {
                classId = 1;
            } else {
                classId = 0; // 背景
            }
            
            // 对于非背景像素，进行颜色叠加
            if (classId > 0 && classId < class_colors.length) {
                const overlayColor = class_colors[classId];
                
                // 使用cv2.addWeighted类似的混合方式 (0.7原图 + 0.3叠加色)
                const alpha = 0.3;
                const beta = 0.7;
                
                originalPixels[i] = Math.min(255, beta * originalPixels[i] + alpha * overlayColor[0]);     // R
                originalPixels[i + 1] = Math.min(255, beta * originalPixels[i + 1] + alpha * overlayColor[1]); // G
                originalPixels[i + 2] = Math.min(255, beta * originalPixels[i + 2] + alpha * overlayColor[2]); // B
                // Alpha通道保持不变
            }
        }
        
        // 绘制混合后的图像
        ctx.putImageData(originalData, 0, 0);
    }
    
    async reviewDefect(approved) {
        if (!this.currentSession || this.currentDefectIndex >= this.defects.length) {
            return;
        }
        
        const defect = this.defects[this.currentDefectIndex];
        
        try {
            const formData = new FormData();
            formData.append('approved', approved);
            
            const response = await fetch(`/api/review/${this.currentSession}/${defect.defect_id}`, {
                method: 'POST',
                body: formData
            });
            
            const result = await response.json();
            
            if (response.ok) {
                // 记录复检结果
                this.reviewResults[defect.defect_id] = approved;
                
                // 更新UI
                this.updateDefectItemStatus(this.currentDefectIndex, approved);
                
                // 显示通知
                const statusText = approved ? '通过' : '不通过';
                window.app.showNotification(`缺陷复检结果: ${statusText}`, 'success');
                
                // 自动跳转到下一个缺陷
                this.nextDefect();
                
            } else {
                window.app.showNotification(`复检失败: ${result.detail}`, 'error');
            }
            
        } catch (error) {
            console.error('复检失败:', error);
            window.app.showNotification(`复检错误: ${error.message}`, 'error');
        }
    }
    
    updateDefectItemStatus(index, approved) {
        const defectItem = document.querySelector(`[data-index="${index}"]`);
        if (defectItem) {
            // 添加复检状态标识
            const statusIcon = approved ? '✅' : '❌';
            const statusClass = approved ? 'reviewed-approved' : 'reviewed-rejected';
            
            defectItem.classList.add(statusClass);
            
            // 在缺陷信息后添加状态图标
            const defectInfo = defectItem.querySelector('.defect-info');
            if (defectInfo && !defectInfo.querySelector('.review-status')) {
                const statusSpan = document.createElement('span');
                statusSpan.className = 'review-status';
                statusSpan.textContent = statusIcon;
                defectInfo.appendChild(statusSpan);
            }
        }
    }
    
    nextDefect() {
        if (this.currentDefectIndex < this.defects.length - 1) {
            this.showDefect(this.currentDefectIndex + 1);
        } else {
            // 所有缺陷都已复检完成
            this.showCompletionMessage();
        }
    }
    
    showCompletionMessage() {
        const reviewPanel = document.getElementById('review-panel');
        reviewPanel.innerHTML = `
            <div style="text-align: center; padding: 2rem; color: rgba(255,255,255,0.8);">
                <h3 style="color: #00d4ff; margin-bottom: 1rem;">🎉 复检完成！</h3>
                <p>所有缺陷都已复检完成。</p>
                <div style="margin-top: 2rem;">
                    <div style="display: flex; justify-content: center; gap: 2rem;">
                        <div>
                            <div style="font-size: 2rem; color: #51cf66;">✅</div>
                            <div>通过: ${Object.values(this.reviewResults).filter(r => r).length}</div>
                        </div>
                        <div>
                            <div style="font-size: 2rem; color: #ff6b6b;">❌</div>
                            <div>不通过: ${Object.values(this.reviewResults).filter(r => !r).length}</div>
                        </div>
                    </div>
                </div>
            </div>
        `;
        
        window.app.showNotification('所有缺陷复检完成！', 'success');
    }
    
    getReviewSummary() {
        const approved = Object.values(this.reviewResults).filter(r => r).length;
        const rejected = Object.values(this.reviewResults).filter(r => !r).length;
        const total = this.defects.length;
        const reviewed = approved + rejected;
        
        return {
            total,
            reviewed,
            approved,
            rejected,
            passRate: reviewed > 0 ? (approved / reviewed * 100).toFixed(1) : 0
        };
    }
}

// 全局变量
window.ReviewManager = ReviewManager;
