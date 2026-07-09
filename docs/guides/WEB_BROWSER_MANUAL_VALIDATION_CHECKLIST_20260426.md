# Web Browser Manual Validation Checklist 2026-04-26

本清单用于补齐 Web Panel 最近 UI 改动的真实浏览器点击验证。它不替代后端测试，也不伪造浏览器通过态；只有人工或 Playwright 在桌面 Chromium 中完成下列步骤后，才能把路线 F 标记为完成。

## 前置条件

- 启动 Web Panel，并确认 `/static/instruction-control.html` 可访问。
- 准备一个测试 session，例如 `browser-validation-20260426`。
- 如需验证审计身份，准备可用的 `Audit Bearer Token`。
- 不接真实硬件时，只允许使用 replay/mock session，不执行现场恢复动作。

可选先运行 Playwright 页面选择器冒烟；它只检查三页关键控件是否可被浏览器定位，不替代后续人工点击：

```bash
python tools/run_web_browser_playwright_smoke.py --base-url http://127.0.0.1:8000 --output test_env/web_browser_manual_validation/playwright_smoke_report.json
```

如果本地未安装 Playwright，runner 会生成 `status=blocked` 和 `playwright_missing`，此时继续按手工清单执行即可。

## 1. Instruction Console

- 打开 `/static/instruction-control.html`。
- 填写 `Session ID`、`Operator`、`Tag`、`Note`。
- 点击发送 instruction set，确认页面显示成功响应。
- 点击发送 simulated circuit config，确认页面显示成功响应。
- 点击生成 recovery plan，确认 `Hardware Recovery` 区域出现 plan 摘要。
- 点击 fault recovery 前确认弹窗内容包含 operator/tag，取消一次并确认页面不下发恢复动作。
- 再次点击 fault recovery 并确认执行，确认 recovery timeline 新增记录。
- 点击 clear faults 前确认弹窗内容包含 operator/tag，确认执行后检查 clear result 摘要。
- 检查节点级状态表包含 `node_id / fault_class / recovery action / recovery status`。
- 检查 failure drill-down 在失败或空数据情况下不阻塞页面。

## 2. Operator History Console

- 打开 `/static/operator-history.html`。
- 使用 session 搜索定位测试 session。
- 分别验证 `operator / tag / note` 过滤。
- 开启 `note_exact` 精确过滤，确认只保留精确匹配记录。
- 切换排序方向，确认列表顺序变化。
- 执行 JSON 导出和 CSV 导出，确认浏览器生成下载或可复制内容。
- 点击 replay，确认 replay payload 与原始 history 条目一致。

## 3. Operator Timeline Compare

- 打开 `/static/operator-history-timeline.html`。
- 使用 session 搜索、operator、tag、note、note exact 过滤。
- 切换 kind 和 route mode 过滤。
- 设置 created after / before 时间范围，确认 timeline 重新加载。
- 选择两条记录，确认 compare 左右面板和 diff summary 更新。
- 点击清空 compare，确认选中状态和 diff 清空。
- 执行 JSON / CSV 导出。

## 4. 响应式与基础可用性

- 在桌面宽度检查三页无横向溢出。
- 在窄屏宽度检查表单、按钮和 pre 区域仍可操作。
- 刷新页面后默认状态不报错。
- 浏览器控制台无阻塞性 JavaScript error。

## 5. 证据归档

完成后归档：

- 浏览器名称与版本
- Web Panel 启动命令
- 测试 session id
- 可选 `playwright_smoke_report.json`
- instruction/recovery/history/timeline 截图
- 导出的 JSON / CSV
- 控制台错误截图或“无阻塞错误”声明

建议输出路径：

```text
test_env/web_browser_manual_validation/
```

可复制 `deployment/web_browser_manual_validation.template.json` 填写实际浏览器证据，并生成机器可审查报告：

```bash
cp deployment/web_browser_manual_validation.template.json test_env/web_browser_manual_validation/browser_validation.json
python tools/build_web_browser_manual_validation_report.py --input test_env/web_browser_manual_validation/browser_validation.json --output test_env/web_browser_manual_validation/web_browser_manual_validation_report.json
```

`evidence.screenshots[]` 和 `evidence.exports[]` 必须填写相对 `browser_validation.json` 所在目录的文件路径，禁止绝对路径和 `..`。报告生成器会检查这些文件真实存在；缺文件或路径越界会使 `web_browser_manual_validation_report.status=blocked`。

最后生成路线 F closeout 判定：

```bash
python tools/build_web_browser_validation_closeout.py --manual-report test_env/web_browser_manual_validation/web_browser_manual_validation_report.json --playwright-report test_env/web_browser_manual_validation/playwright_smoke_report.json --output test_env/web_browser_manual_validation/web_browser_validation_closeout.json
```

再生成发布归档 evidence pack：

```bash
python tools/build_web_browser_validation_evidence_pack.py --manual-report test_env/web_browser_manual_validation/web_browser_manual_validation_report.json --closeout test_env/web_browser_manual_validation/web_browser_validation_closeout.json --playwright-report test_env/web_browser_manual_validation/playwright_smoke_report.json --output test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json
```

如果本次发布要求 Playwright 也必须通过，可加 `--require-playwright`。

`playwright_smoke_report.json` 是支持证据；如果 Playwright 未安装而报告为 `blocked`，closeout 会记录 warning。`web_browser_manual_validation_report.json` 必须是 `status=passed`，路线 F 才能关闭。

完成判定：

- instruction set、simulated circuit、recovery plan、recover、clear faults 均完成一次浏览器操作。
- history 搜索、精确 note 过滤、排序、导出、replay 均完成一次浏览器操作。
- timeline compare、过滤、导出均完成一次浏览器操作。
- 响应式和控制台检查无阻塞问题。
- `web_browser_manual_validation_report.json` 的 `status=passed`。
- `web_browser_validation_closeout.json` 的 `status=passed`。
- `web_browser_validation_evidence_pack.json` 的 `status=ready`。
