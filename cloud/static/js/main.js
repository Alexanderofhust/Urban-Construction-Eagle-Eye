// 主应用逻辑
class EagleEyeApp {
    constructor() {
        this.currentSession = null;
        this.currentPage = 'dashboard';
        this.connectionStatus = 'disconnected';
        this.progressInterval = null;
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
        this.loadSessions();
        this.updateStatus();
    }
    
    setupEventListeners() {
        // 导航按钮事件
        document.querySelectorAll('.nav-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const page = e.target.dataset.page;
                this.switchPage(page);
            });
        });
        
        // 连接表单事件
        const connectionForm = document.getElementById('connection-form');
        if (connectionForm) {
            connectionForm.addEventListener('submit', (e) => {
                e.preventDefault();
                this.handleConnection();
            });
        }
        
        // 会话选择事件
        document.addEventListener('change', (e) => {
            if (e.target.classList.contains('session-selector')) {
                this.currentSession = e.target.value;
                this.updateSessionSelectors();
            }
        });
        
        // 定期更新状态
        setInterval(() => {
            this.updateStatus();
        }, 2000);
    }
    
    switchPage(page) {
        // 更新导航按钮状态
        document.querySelectorAll('.nav-btn').forEach(btn => {
            btn.classList.remove('active');
        });
        document.querySelector(`[data-page="${page}"]`).classList.add('active');
        
        // 切换页面显示
        document.querySelectorAll('.page').forEach(p => {
            p.classList.remove('active');
        });
        document.getElementById(`${page}-page`).classList.add('active');
        
        this.currentPage = page;
        
        // 页面特定初始化
        switch(page) {
            case 'pointcloud':
                this.initPointcloudPage();
                break;
            case 'review':
                this.initReviewPage();
                break;
            case 'analysis':
                this.initAnalysisPage();
                break;
            case 'statistics':
                this.initStatisticsPage();
                break;
        }
    }
    
    async handleConnection() {
        const form = document.getElementById('connection-form');
        const formData = new FormData(form);
        const connectBtn = document.getElementById('connect-btn');
        const progressContainer = document.getElementById('progress-container');
        
        try {
            // 更新UI状态
            connectBtn.disabled = true;
            connectBtn.querySelector('.btn-text').style.display = 'none';
            connectBtn.querySelector('.btn-loading').style.display = 'inline';
            progressContainer.style.display = 'block';
            
            this.connectionStatus = 'connecting';
            this.updateConnectionStatus();
            
            // 发送连接请求
            const response = await fetch('/api/connect', {
                method: 'POST',
                body: formData
            });
            
            const result = await response.json();
            
            if (response.ok) {
                this.connectionStatus = 'connected';
                this.currentSession = result.session_id;
                this.showNotification('连接成功！数据下载完成', 'success');
                
                // 刷新会话列表
                await this.loadSessions();
                
                // 更新所有会话选择器
                this.updateSessionSelectors();
                
            } else {
                this.connectionStatus = 'disconnected';
                this.showNotification(`连接失败: ${result.detail}`, 'error');
            }
            
        } catch (error) {
            this.connectionStatus = 'disconnected';
            this.showNotification(`连接错误: ${error.message}`, 'error');
        } finally {
            // 重置UI状态
            connectBtn.disabled = false;
            connectBtn.querySelector('.btn-text').style.display = 'inline';
            connectBtn.querySelector('.btn-loading').style.display = 'none';
            progressContainer.style.display = 'none';
            
            this.updateConnectionStatus();
        }
    }
    
    async loadSessions() {
        try {
            const response = await fetch('/api/sessions');
            const data = await response.json();
            
            if (response.ok) {
                this.renderSessions(data.sessions);
                this.updateQuickStats(data.sessions);
            }
        } catch (error) {
            console.error('加载会话失败:', error);
        }
    }
    
    renderSessions(sessions) {
        const sessionsList = document.getElementById('sessions-list');
        
        if (!sessions || sessions.length === 0) {
            sessionsList.innerHTML = '<p style="text-align: center; color: rgba(255,255,255,0.7); padding: 2rem;">暂无会话数据</p>';
            return;
        }
        
        sessionsList.innerHTML = sessions.map(session => `
            <div class="session-item" data-session-id="${session.session_id}">
                <div class="session-header">
                    <div class="session-id">${session.session_id}</div>
                    <div class="session-time">${new Date(session.created_time).toLocaleString()}</div>
                </div>
                <div class="session-info">
                    <span>📸 ${session.image_count} 图像</span>
                    <span>🎭 ${session.mask_count} 掩码</span>
                    <span>📍 ${session.has_location ? '有GPS' : '无GPS'}</span>
                    <span>☁️ ${session.has_pointcloud ? '有点云' : '无点云'}</span>
                </div>
            </div>
        `).join('');
        
        // 添加会话点击事件
        sessionsList.querySelectorAll('.session-item').forEach(item => {
            item.addEventListener('click', (e) => {
                const sessionId = e.currentTarget.dataset.sessionId;
                this.selectSession(sessionId);
            });
        });
    }
    
    selectSession(sessionId) {
        this.currentSession = sessionId;
        
        // 更新会话列表UI
        document.querySelectorAll('.session-item').forEach(item => {
            item.classList.remove('active');
        });
        document.querySelector(`[data-session-id="${sessionId}"]`).classList.add('active');
        
        // 更新所有会话选择器
        this.updateSessionSelectors();
        
        // 加载地图数据
        if (window.loadLocationData) {
            window.loadLocationData(sessionId);
        }
        
        this.showNotification(`已选择会话: ${sessionId}`, 'info');
    }
    
    updateSessionSelectors() {
        const selectors = document.querySelectorAll('.session-selector, #session-selector, #review-session-selector, #analysis-session-selector, #stats-session-selector');
        
        selectors.forEach(selector => {
            if (this.currentSession) {
                // 检查选项是否存在
                const option = selector.querySelector(`option[value="${this.currentSession}"]`);
                if (option) {
                    selector.value = this.currentSession;
                }
            }
        });
    }
    
    updateQuickStats(sessions) {
        const totalSessions = sessions.length;
        const totalDefects = sessions.reduce((sum, session) => sum + session.mask_count, 0);
        const reviewedCount = 0; // 这里应该从后端获取复检数量
        
        document.getElementById('total-sessions').textContent = totalSessions;
        document.getElementById('total-defects').textContent = totalDefects;
        document.getElementById('reviewed-count').textContent = reviewedCount;
    }
    
    updateStatus() {
        this.updateConnectionStatus();
        
        // 如果正在连接，更新进度
        if (this.connectionStatus === 'connecting') {
            this.updateProgress();
        }
    }
    
    updateConnectionStatus() {
        const statusElement = document.getElementById('connection-status');
        
        statusElement.className = `status-indicator ${this.connectionStatus}`;
        
        switch(this.connectionStatus) {
            case 'connected':
                statusElement.textContent = '已连接';
                break;
            case 'connecting':
                statusElement.textContent = '连接中';
                break;
            case 'disconnected':
            default:
                statusElement.textContent = '未连接';
                break;
        }
    }
    
    async updateProgress() {
        try {
            const response = await fetch('/api/status');
            const data = await response.json();
            
            if (data.download_progress !== undefined) {
                const progressFill = document.getElementById('progress-fill');
                const progressText = document.getElementById('progress-text');
                
                if (progressFill && progressText) {
                    progressFill.style.width = `${data.download_progress}%`;
                    progressText.textContent = `${data.download_progress}%`;
                }
            }
        } catch (error) {
            console.error('更新进度失败:', error);
        }
    }
    
    showNotification(message, type = 'info') {
        // 创建通知元素
        const notification = document.createElement('div');
        notification.className = `notification notification-${type}`;
        notification.textContent = message;
        
        // 添加样式
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 1rem 1.5rem;
            border-radius: 8px;
            color: white;
            font-weight: 500;
            z-index: 10000;
            animation: slideIn 0.3s ease-out;
        `;
        
        // 设置背景色
        switch(type) {
            case 'success':
                notification.style.background = 'rgba(81, 207, 102, 0.9)';
                break;
            case 'error':
                notification.style.background = 'rgba(255, 107, 107, 0.9)';
                break;
            case 'info':
            default:
                notification.style.background = 'rgba(0, 212, 255, 0.9)';
                break;
        }
        
        // 添加动画样式
        const style = document.createElement('style');
        style.textContent = `
            @keyframes slideIn {
                from { transform: translateX(100%); opacity: 0; }
                to { transform: translateX(0); opacity: 1; }
            }
        `;
        document.head.appendChild(style);
        
        document.body.appendChild(notification);
        
        // 3秒后自动移除
        setTimeout(() => {
            notification.remove();
        }, 3000);
    }
    
    // 页面初始化方法
    initPointcloudPage() {
        if (window.PointcloudViewer) {
            window.pointcloudViewer = new PointcloudViewer();
        }
    }
    
    initReviewPage() {
        if (window.ReviewManager) {
            window.reviewManager = new ReviewManager();
        }
    }
    
    initAnalysisPage() {
        if (window.AnalysisManager) {
            window.analysisManager = new AnalysisManager();
        }
    }
    
    initStatisticsPage() {
        if (window.StatisticsManager) {
            window.statisticsManager = new StatisticsManager();
        }
    }
}

// 应用程序入口
document.addEventListener('DOMContentLoaded', () => {
    // 创建全局应用实例
    window.app = new EagleEyeApp();
    
    // 初始化会话选择器
    const initSessionSelector = async () => {
        try {
            const response = await fetch('/api/sessions');
            const data = await response.json();
            
            if (response.ok && data.sessions) {
                const selectors = document.querySelectorAll('.session-selector, #session-selector, #review-session-selector, #analysis-session-selector, #stats-session-selector');
                
                selectors.forEach(selector => {
                    // 清空现有选项（除了第一个默认选项）
                    const defaultOption = selector.querySelector('option[value=""]');
                    selector.innerHTML = '';
                    if (defaultOption) {
                        selector.appendChild(defaultOption);
                    }
                    
                    // 添加会话选项
                    data.sessions.forEach(session => {
                        const option = document.createElement('option');
                        option.value = session.session_id;
                        option.textContent = `${session.session_id} (${new Date(session.created_time).toLocaleDateString()})`;
                        selector.appendChild(option);
                    });
                });
            }
        } catch (error) {
            console.error('初始化会话选择器失败:', error);
        }
    };
    
    initSessionSelector();
    
    // 定期刷新会话选择器
    setInterval(initSessionSelector, 30000);
});

// 工具函数
window.utils = {
    formatFileSize: (bytes) => {
        if (bytes === 0) return '0 B';
        const k = 1024;
        const sizes = ['B', 'KB', 'MB', 'GB'];
        const i = Math.floor(Math.log(bytes) / Math.log(k));
        return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
    },
    
    formatDate: (dateString) => {
        return new Date(dateString).toLocaleString('zh-CN');
    },
    
    debounce: (func, wait) => {
        let timeout;
        return function executedFunction(...args) {
            const later = () => {
                clearTimeout(timeout);
                func(...args);
            };
            clearTimeout(timeout);
            timeout = setTimeout(later, wait);
        };
    }
};
