// AI分析管理器
class AnalysisManager {
    constructor() {
        this.currentSession = null;
        this.analysisResult = null;
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
    }
    
    setupEventListeners() {
        // 开始分析按钮
        const startBtn = document.getElementById('start-analysis-btn');
        if (startBtn) {
            startBtn.addEventListener('click', () => {
                this.startAnalysis();
            });
        }
        
        // 会话选择器
        const sessionSelector = document.getElementById('analysis-session-selector');
        if (sessionSelector) {
            sessionSelector.addEventListener('change', (e) => {
                this.currentSession = e.target.value;
            });
        }
    }
    
    async startAnalysis() {
        if (!this.currentSession) {
            window.app.showNotification('请先选择一个会话', 'error');
            return;
        }
        
        try {
            const startBtn = document.getElementById('start-analysis-btn');
            startBtn.disabled = true;
            startBtn.textContent = '分析中...';
            
            // 显示加载界面
            this.showLoadingState();
            
            // 发送分析请求
            const response = await fetch(`/api/analyze/${this.currentSession}`, {
                method: 'POST'
            });
            
            const data = await response.json();
            
            if (response.ok) {
                this.analysisResult = data.analysis;
                this.showAnalysisResults();
                window.app.showNotification('AI分析完成', 'success');
            } else {
                this.hideLoadingState();
                window.app.showNotification(`分析失败: ${data.detail}`, 'error');
            }
            
        } catch (error) {
            console.error('AI分析失败:', error);
            this.hideLoadingState();
            window.app.showNotification(`分析错误: ${error.message}`, 'error');
        } finally {
            const startBtn = document.getElementById('start-analysis-btn');
            startBtn.disabled = false;
            startBtn.textContent = '开始分析';
        }
    }
    
    showLoadingState() {
        const loadingContainer = document.getElementById('analysis-loading');
        const resultsContainer = document.getElementById('analysis-results');
        
        loadingContainer.style.display = 'flex';
        resultsContainer.style.display = 'none';
    }
    
    hideLoadingState() {
        const loadingContainer = document.getElementById('analysis-loading');
        loadingContainer.style.display = 'none';
    }
    
    showAnalysisResults() {
        const loadingContainer = document.getElementById('analysis-loading');
        const resultsContainer = document.getElementById('analysis-results');
        
        loadingContainer.style.display = 'none';
        resultsContainer.style.display = 'block';
        
        // 更新分析结果
        this.updateAnalysisResults();
    }
    
    updateAnalysisResults() {
        if (!this.analysisResult) return;
        
        // 更新状况评级
        const conditionBadge = document.getElementById('condition-badge');
        const conditionText = document.getElementById('condition-text');
        const conditionScore = document.getElementById('condition-score');
        
        if (conditionBadge && conditionText && conditionScore) {
            const condition = this.analysisResult.overall_condition || '未知';
            const score = this.analysisResult.condition_score || 0;
            
            conditionText.textContent = condition;
            conditionScore.textContent = score;
            
            // 设置状况样式
            conditionBadge.className = `condition-badge ${this.getConditionClass(score)}`;
        }
        
        // 更新建筑物描述
        const descriptionElement = document.getElementById('building-description');
        if (descriptionElement) {
            descriptionElement.textContent = this.analysisResult.description || '暂无描述';
        }
        
        // 更新发现的缺陷
        const defectsFoundElement = document.getElementById('defects-found');
        if (defectsFoundElement) {
            const defects = this.analysisResult.defects || [];
            defectsFoundElement.innerHTML = defects.map(defect => 
                `<li>${defect}</li>`
            ).join('');
        }
        
        // 更新维修建议
        const repairSuggestionsElement = document.getElementById('repair-suggestions');
        if (repairSuggestionsElement) {
            const suggestions = this.analysisResult.repair_suggestions || [];
            repairSuggestionsElement.innerHTML = suggestions.map(suggestion => 
                `<li>${suggestion}</li>`
            ).join('');
        }
        
        // 更新紧急程度
        const urgencyIndicator = document.getElementById('urgency-indicator');
        const urgencyLevel = document.getElementById('urgency-level');
        
        if (urgencyIndicator && urgencyLevel) {
            const urgency = this.analysisResult.urgency_level || '未知';
            urgencyLevel.textContent = urgency;
            urgencyIndicator.className = `urgency-indicator ${this.getUrgencyClass(urgency)}`;
        }
    }
    
    getConditionClass(score) {
        if (score >= 9) return 'excellent';
        if (score >= 7) return 'good';
        if (score >= 5) return 'fair';
        if (score >= 3) return 'poor';
        return 'dangerous';
    }
    
    getUrgencyClass(urgency) {
        switch(urgency.toLowerCase()) {
            case '高':
            case 'high':
                return 'high';
            case '中':
            case 'medium':
                return 'medium';
            case '低':
            case 'low':
                return 'low';
            default:
                return 'medium';
        }
    }
    
    generateReport() {
        if (!this.analysisResult) {
            window.app.showNotification('没有可用的分析结果', 'error');
            return;
        }
        
        const reportContent = this.createReportContent();
        this.downloadReport(reportContent);
    }
    
    createReportContent() {
        const timestamp = new Date().toLocaleString('zh-CN');
        
        return `
# 建筑缺陷AI分析报告

**生成时间:** ${timestamp}
**会话ID:** ${this.currentSession}

## 分析概要

**建筑状况评级:** ${this.analysisResult.overall_condition}
**状况评分:** ${this.analysisResult.condition_score}/10
**紧急程度:** ${this.analysisResult.urgency_level}

## 建筑物描述

${this.analysisResult.description}

## 发现的缺陷

${this.analysisResult.defects.map(defect => `- ${defect}`).join('\n')}

## 维修建议

${this.analysisResult.repair_suggestions.map(suggestion => `- ${suggestion}`).join('\n')}

## 详细分析

${this.analysisResult.detailed_analysis}

---

*此报告由城市建设鹰眼系统AI分析生成*
        `.trim();
    }
    
    downloadReport(content) {
        const blob = new Blob([content], { type: 'text/markdown;charset=utf-8;' });
        const link = document.createElement('a');
        
        if (link.download !== undefined) {
            const url = URL.createObjectURL(blob);
            link.setAttribute('href', url);
            link.setAttribute('download', `建筑缺陷分析报告_${this.currentSession}_${new Date().toISOString().split('T')[0]}.md`);
            link.style.visibility = 'hidden';
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            
            window.app.showNotification('报告下载成功', 'success');
        } else {
            window.app.showNotification('浏览器不支持文件下载', 'error');
        }
    }
    
    // 获取分析结果摘要
    getAnalysisSummary() {
        if (!this.analysisResult) return null;
        
        return {
            condition: this.analysisResult.overall_condition,
            score: this.analysisResult.condition_score,
            urgency: this.analysisResult.urgency_level,
            defectsCount: this.analysisResult.defects?.length || 0,
            suggestionsCount: this.analysisResult.repair_suggestions?.length || 0
        };
    }
    
    // 清空分析结果
    clearResults() {
        this.analysisResult = null;
        
        const resultsContainer = document.getElementById('analysis-results');
        resultsContainer.style.display = 'none';
        
        window.app.showNotification('分析结果已清空', 'info');
    }
}

// 全局变量
window.AnalysisManager = AnalysisManager;
