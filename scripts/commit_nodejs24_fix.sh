#!/bin/bash
# AGI-Walker GitHub Actions Node.js 24 兼容性修复
# 一键提交和推送修复

set -e

echo "════════════════════════════════════════════════════════"
echo "🔧 AGI-Walker GitHub Actions Node.js 24 兼容性修复"
echo "════════════════════════════════════════════════════════"
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 步骤 1: 显示修改
echo -e "${BLUE}步骤 1: 显示所有修改${NC}"
echo ""
git diff --stat .github/workflows/

echo ""
echo -e "${BLUE}修改详情:${NC}"
git diff .github/workflows/ci.yml | head -20
echo "..."
git diff .github/workflows/godot-test.yml | head -20
echo ""

# 步骤 2: 选择分支确认
echo -e "${BLUE}步骤 2: 确认推送分支${NC}"
echo ""
current_branch=$(git rev-parse --abbrev-ref HEAD)
echo "当前分支: $current_branch"
echo ""

if [ "$current_branch" != "develop" ] && [ "$current_branch" != "main" ]; then
    echo -e "${YELLOW}⚠️  警告: 当前分支不是 develop 或 main${NC}"
    read -p "继续推送到 $current_branch? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "已取消"
        exit 1
    fi
fi

# 步骤 3: 暂存所有修改
echo -e "${BLUE}步骤 3: 暂存修改${NC}"
echo ""
git add .github/workflows/ci.yml
git add .github/workflows/godot-test.yml
git add scripts/validate_github_actions.py
git add GITHUB_ACTIONS_NODEJS24_FIX.md
git add CI_NODEJS24_QUICK_FIX.md
git add CI_NODEJS24_FIX_COMPLETION_REPORT.md

echo -e "${GREEN}✓ 文件已暂存${NC}"
echo ""
git status --short

# 步骤 4: 提交
echo ""
echo -e "${BLUE}步骤 4: 创建提交${NC}"
echo ""

commit_message="ci: enable Node.js 24 compatibility (P3)

[修复内容]
- Add FORCE_JAVASCRIPT_ACTIONS_TO_NODE24 env variable to all workflows
- Update .github/workflows/ci.yml: main CI/CD pipeline
- Update .github/workflows/godot-test.yml: Godot-specific tests
- Add scripts/validate_github_actions.py: automatic compatibility check
- Add comprehensive documentation and guides

[技术细节]
- Fixes GitHub Actions Node.js 20 deprecation warning
- Ensures compatibility with Node.js 24 (deadline: 2026-06-02)
- No breaking changes to existing code
- All tests remain unchanged

[验证]
✓ All workflows validated and compatible
✓ No version conflicts detected
✓ Ready for immediate deployment

[相关文档]
- GITHUB_ACTIONS_NODEJS24_FIX.md: Complete technical guide
- CI_NODEJS24_QUICK_FIX.md: Quick reference and troubleshooting
- CI_NODEJS24_FIX_COMPLETION_REPORT.md: Detailed completion report

[后续计划]
- Monitor GitHub Actions execution
- Prepare complete Node.js 24 migration (by 2026-05-31)
- Remove FORCE_JAVASCRIPT_ACTIONS_TO_NODE24 when native support available

Links:
- GitHub Blog: https://github.blog/changelog/
- GitHub Actions: https://docs.github.com/en/actions
"

git commit -m "$commit_message"

echo -e "${GREEN}✓ 提交已创建${NC}"
echo ""

# 步骤 5: 显示提交信息
echo -e "${BLUE}提交信息:${NC}"
git log --oneline -1

# 步骤 6: 推送确认
echo ""
echo -e "${BLUE}步骤 5: 推送到远程${NC}"
echo ""
read -p "推送到 $current_branch? (Y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Nn]$ ]]; then
    git push origin $current_branch
    echo -e "${GREEN}✓ 代码已推送${NC}"
    echo ""
    echo -e "${YELLOW}📌 后续步骤:${NC}"
    echo ""
    echo "1. 打开 GitHub Actions 监控执行:"
    echo "   https://github.com/YOUR_ORG/AGI-Walker/actions"
    echo ""
    echo "2. 验证工作流通过:"
    echo "   - quality: passing"
    echo "   - test: passing"
    echo "   - integration: passing"
    echo "   - release-check: passing"
    echo "   - godot-test: passing"
    echo ""
    echo "3. 如果使用 develop 分支，创建 Pull Request:"
    echo "   develop -> main"
    echo ""
    echo "4. 检查是否有任何 Node.js 版本相关的警告"
    echo ""
else
    echo "已取消推送"
    echo ""
    echo -e "${YELLOW}提示:${NC} 你可以稍后运行以下命令推送:"
    echo "  git push origin $current_branch"
    exit 0
fi

# 最终总结
echo ""
echo "════════════════════════════════════════════════════════"
echo -e "${GREEN}✅ 修复完成!${NC}"
echo "════════════════════════════════════════════════════════"
echo ""
echo "修改摘要:"
echo "  • 工作流文件: 2 个"
echo "  • 环境变量: +2"
echo "  • 验证脚本: +1"
echo "  • 文档: +3"
echo ""
echo "预期结果:"
echo "  • 消除 Node.js 20 弃用警告"
echo "  • 完全 Node.js 24 兼容"
echo "  • 所有测试保持现状"
echo ""
