# 模型权重目录

按照《需求文档.md》的项目结构，运行时使用的 YOLO 权重放在本目录：

```text
data/weights/best.pt
```

权重文件通常较大，已由仓库根目录的 `.gitignore` 排除，不会被普通 Git 提交。
请从训练产物或受信任的独立下载地址取得 `best.pt`，放入本目录后设置：

```bash
export AGRI_TOMATO_MODEL=/home/fsy/Documents/Codex/Agricultural-Bot/Agricultural_Bot/data/weights/best.pt
```

也可以把模型放在项目外的任意位置，并在启动命令中传入绝对路径。
