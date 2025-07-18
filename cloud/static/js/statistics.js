// 统计管理器
class StatisticsManager {
    constructor() {
        this.currentSession = null;
        this.statsData = null;
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
    }
    
    setupEventListeners() {
        // 生成报告按钮
        const generateBtn = document.getElementById('generate-report-btn');
        if (generateBtn) {
            generateBtn.addEventListener('click', () => {
                this.generateReport();
            });
        }
        
        // 会话选择器
        const sessionSelector = document.getElementById('stats-session-selector');
        if (sessionSelector) {
            sessionSelector.addEventListener('change', (e) => {
                this.currentSession = e.target.value;
            });
        }
    }
    
    async generateReport() {
        if (!this.currentSession) {
            window.app.showNotification('请先选择一个会话', 'error');
            return;
        }
        
        try {
            const generateBtn = document.getElementById('generate-report-btn');
            generateBtn.disabled = true;
            generateBtn.textContent = '生成中...';
            
            // 获取统计数据
            const response = await fetch(`/api/statistics/${this.currentSession}`);
            const data = await response.json();
            
            if (response.ok) {
                this.statsData = data.statistics;
                this.updateStatisticsDisplay();
                window.app.showNotification('统计报告生成完成', 'success');
            } else {
                window.app.showNotification(`生成失败: ${data.detail}`, 'error');
            }
            
        } catch (error) {
            console.error('生成统计报告失败:', error);
            window.app.showNotification(`生成错误: ${error.message}`, 'error');
        } finally {
            const generateBtn = document.getElementById('generate-report-btn');
            generateBtn.disabled = false;
            generateBtn.textContent = '生成报告';
        }
    }
    
    updateStatisticsDisplay() {
        if (!this.statsData) return;
        
        // 更新基础统计
        this.updateBasicStats();
        
        // 更新缺陷分布
        this.updateDefectDistribution();
        
        // 如果有AI分析结果，显示相关统计
        if (this.statsData.ai_analysis) {
            this.updateAIAnalysisStats();
        }
    }
    
    updateBasicStats() {
        // 总图像数
        const totalImagesElement = document.getElementById('stat-total-images');
        if (totalImagesElement) {
            totalImagesElement.textContent = this.statsData.total_images || 0;
        }
        
        // 总缺陷数
        const totalDefectsElement = document.getElementById('stat-total-defects');
        if (totalDefectsElement) {
            totalDefectsElement.textContent = this.statsData.total_defects || 0;
        }
        
        // 已复检数
        const reviewedElement = document.getElementById('stat-reviewed');
        if (reviewedElement) {
            reviewedElement.textContent = this.statsData.reviewed_count || 0;
        }
        
        // 通过率
        const passRateElement = document.getElementById('stat-pass-rate');
        if (passRateElement) {
            const passRate = this.statsData.reviewed_count > 0 
                ? ((this.statsData.approved_count / this.statsData.reviewed_count) * 100).toFixed(1)
                : 0;
            passRateElement.textContent = `${passRate}%`;
        }
    }
    
    updateDefectDistribution() {
        // 红色缺陷（严重）
        const redDefectsElement = document.getElementById('red-defects');
        if (redDefectsElement) {
            redDefectsElement.textContent = this.statsData.defect_types?.red || 0;
        }
        
        // 蓝色缺陷（中等）
        const blueDefectsElement = document.getElementById('blue-defects');
        if (blueDefectsElement) {
            blueDefectsElement.textContent = this.statsData.defect_types?.blue || 0;
        }
        
        // 绿色缺陷（轻微）
        const greenDefectsElement = document.getElementById('green-defects');
        if (greenDefectsElement) {
            greenDefectsElement.textContent = this.statsData.defect_types?.green || 0;
        }
    }
    
    updateAIAnalysisStats() {
        const aiAnalysis = this.statsData.ai_analysis;
        
        // 如果页面上有AI分析相关的统计显示区域，在这里更新
        // 由于当前HTML模板中没有相关元素，这里预留接口
        
        console.log('AI分析统计:', aiAnalysis);
    }
    
    exportStatistics() {
        if (!this.statsData) {
            window.app.showNotification('没有可用的统计数据', 'error');
            return;
        }
        
        const exportData = {
            session_id: this.currentSession,
            timestamp: new Date().toISOString(),
            statistics: this.statsData
        };
        
        // 导出为JSON
        this.downloadJSON(exportData, `统计数据_${this.currentSession}.json`);
    }
    
    exportCSV() {
        if (!this.statsData) {
            window.app.showNotification('没有可用的统计数据', 'error');
            return;
        }
        
        const csvData = this.convertToCSV(this.statsData);
        this.downloadCSV(csvData, `统计数据_${this.currentSession}.csv`);
    }
    
    convertToCSV(data) {
        const rows = [
            ['指标', '数值'],
            ['会话ID', data.session_id],
            ['总图像数', data.total_images],
            ['总掩码数', data.total_masks],
            ['总缺陷数', data.total_defects],
            ['已复检数', data.reviewed_count],
            ['通过数', data.approved_count],
            ['未通过数', data.rejected_count],
            ['通过率', data.reviewed_count > 0 ? ((data.approved_count / data.reviewed_count) * 100).toFixed(1) + '%' : '0%'],
            ['红色缺陷', data.defect_types?.red || 0],
            ['蓝色缺陷', data.defect_types?.blue || 0],
            ['绿色缺陷', data.defect_types?.green || 0]
        ];
        
        if (data.ai_analysis) {
            rows.push(['AI评分', data.ai_analysis.condition_score]);
            rows.push(['整体状况', data.ai_analysis.overall_condition]);
            rows.push(['紧急程度', data.ai_analysis.urgency_level]);
        }
        
        return rows.map(row => row.join(',')).join('\n');
    }
    
    downloadJSON(data, filename) {
        const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
        this.downloadBlob(blob, filename);
    }
    
    downloadCSV(data, filename) {
        const blob = new Blob(['\ufeff' + data], { type: 'text/csv;charset=utf-8;' });
        this.downloadBlob(blob, filename);
    }
    
    downloadBlob(blob, filename) {
        const link = document.createElement('a');
        
        if (link.download !== undefined) {
            const url = URL.createObjectURL(blob);
            link.setAttribute('href', url);
            link.setAttribute('download', filename);
            link.style.visibility = 'hidden';
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            
            window.app.showNotification('文件下载成功', 'success');
        } else {
            window.app.showNotification('浏览器不支持文件下载', 'error');
        }
    }
    
    generateDetailedReport() {
        if (!this.statsData) {
            window.app.showNotification('没有可用的统计数据', 'error');
            return;
        }
        
        const reportContent = this.createDetailedReportContent();
        const blob = new Blob([reportContent], { type: 'text/markdown;charset=utf-8;' });
        this.downloadBlob(blob, `详细统计报告_${this.currentSession}_${new Date().toISOString().split('T')[0]}.md`);
    }
    
    createDetailedReportContent() {
        const timestamp = new Date().toLocaleString('zh-CN');
        const passRate = this.statsData.reviewed_count > 0 
            ? ((this.statsData.approved_count / this.statsData.reviewed_count) * 100).toFixed(1)
            : 0;
        
        let content = `
# 建筑缺陷检测统计报告

**生成时间:** ${timestamp}
**会话ID:** ${this.currentSession}

## 数据概览

| 指标 | 数值 |
|------|------|
| 总图像数 | ${this.statsData.total_images} |
| 总掩码数 | ${this.statsData.total_masks} |
| 发现缺陷数 | ${this.statsData.total_defects} |
| 已复检数 | ${this.statsData.reviewed_count} |
| 通过数 | ${this.statsData.approved_count} |
| 未通过数 | ${this.statsData.rejected_count} |
| 通过率 | ${passRate}% |

## 缺陷分布

| 缺陷类型 | 数量 | 占比 |
|----------|------|------|
| 严重缺陷 (红色) | ${this.statsData.defect_types?.red || 0} | ${this.calculatePercentage(this.statsData.defect_types?.red || 0)}% |
| 中等缺陷 (蓝色) | ${this.statsData.defect_types?.blue || 0} | ${this.calculatePercentage(this.statsData.defect_types?.blue || 0)}% |
| 轻微缺陷 (绿色) | ${this.statsData.defect_types?.green || 0} | ${this.calculatePercentage(this.statsData.defect_types?.green || 0)}% |

## 复检情况

- **复检进度:** ${this.statsData.reviewed_count}/${this.statsData.total_defects} (${this.calculateReviewProgress()}%)
- **复检通过率:** ${passRate}%
- **待复检数量:** ${this.statsData.total_defects - this.statsData.reviewed_count}
`;

        // 如果有AI分析结果
        if (this.statsData.ai_analysis) {
            content += `

## AI分析结果

- **整体状况:** ${this.statsData.ai_analysis.overall_condition}
- **状况评分:** ${this.statsData.ai_analysis.condition_score}/10
- **紧急程度:** ${this.statsData.ai_analysis.urgency_level}
`;
        }
        
        content += `

## 数据质量

- **图像-掩码匹配率:** ${this.calculateMatchRate()}%
- **数据完整性:** ${this.calculateDataIntegrity()}%

---

*此报告由城市建设鹰眼系统自动生成*
        `;
        
        return content.trim();
    }
    
    calculatePercentage(value) {
        const total = (this.statsData.defect_types?.red || 0) + 
                     (this.statsData.defect_types?.blue || 0) + 
                     (this.statsData.defect_types?.green || 0);
        return total > 0 ? ((value / total) * 100).toFixed(1) : 0;
    }
    
    calculateReviewProgress() {
        return this.statsData.total_defects > 0 
            ? ((this.statsData.reviewed_count / this.statsData.total_defects) * 100).toFixed(1)
            : 0;
    }
    
    calculateMatchRate() {
        // 假设图像和掩码应该匹配
        const matchRate = this.statsData.total_images > 0 
            ? Math.min(this.statsData.total_masks / this.statsData.total_images, 1) * 100
            : 0;
        return matchRate.toFixed(1);
    }
    
    calculateDataIntegrity() {
        // 简单的数据完整性计算
        let score = 0;
        let maxScore = 0;
        
        // 检查是否有图像数据
        if (this.statsData.total_images > 0) score += 25;
        maxScore += 25;
        
        // 检查是否有掩码数据
        if (this.statsData.total_masks > 0) score += 25;
        maxScore += 25;
        
        // 检查是否有缺陷数据
        if (this.statsData.total_defects > 0) score += 25;
        maxScore += 25;
        
        // 检查是否有复检数据
        if (this.statsData.reviewed_count > 0) score += 25;
        maxScore += 25;
        
        return maxScore > 0 ? ((score / maxScore) * 100).toFixed(1) : 0;
    }
    
    // 获取统计摘要
    getStatsSummary() {
        if (!this.statsData) return null;
        
        return {
            totalImages: this.statsData.total_images,
            totalDefects: this.statsData.total_defects,
            reviewedCount: this.statsData.reviewed_count,
            approvedCount: this.statsData.approved_count,
            passRate: this.statsData.reviewed_count > 0 
                ? ((this.statsData.approved_count / this.statsData.reviewed_count) * 100).toFixed(1)
                : 0,
            defectTypes: this.statsData.defect_types
        };
    }
    
    // 清空统计数据
    clearStats() {
        this.statsData = null;
        
        // 重置显示
        document.getElementById('stat-total-images').textContent = '-';
        document.getElementById('stat-total-defects').textContent = '-';
        document.getElementById('stat-reviewed').textContent = '-';
        document.getElementById('stat-pass-rate').textContent = '-';
        document.getElementById('red-defects').textContent = '0';
        document.getElementById('blue-defects').textContent = '0';
        document.getElementById('green-defects').textContent = '0';
        
        window.app.showNotification('统计数据已清空', 'info');
    }
}

// 全局变量
window.StatisticsManager = StatisticsManager;
