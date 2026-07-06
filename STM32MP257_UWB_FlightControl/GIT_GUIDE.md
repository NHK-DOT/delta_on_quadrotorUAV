# Git 同步命令教程

本项目已配置自动推送到 GitHub（`post-commit hook`），日常开发只需 `add + commit` 即可。

## 日常工作流

```bash
# 1. 查看当前修改状态
git status

# 2. 添加修改到暂存区
git add .                    # 添加所有修改
git add <file>               # 添加指定文件

# 3. 提交（会自动推送到 GitHub）
git commit -m "描述你做了什么"

# 4. 查看提交历史
git log --oneline
```

## 分支操作

```bash
# 查看所有分支
git branch -a

# 创建并切换到新分支
git checkout -b feature/xxx

# 切换回 main 分支
git checkout main

# 合并分支到 main
git merge feature/xxx

# 删除已合并的分支
git branch -d feature/xxx
```

## 版本回退

```bash
# 查看提交历史（找到想回退的 commit hash）
git log --oneline

# 方式一：安全回退（推荐，生成一个新的"撤销"提交）
git revert <commit-hash>

# 方式二：强制回退（谨慎使用，会丢失之后的提交）
git reset --hard <commit-hash>

# 如果强制回退后需要推送到远程
git push --force
```

## 远程同步

```bash
# 拉取远程最新代码
git pull origin main

# 手动推送（通常不需要，commit 会自动推送）
git push origin main

# 查看远程仓库信息
git remote -v
```

## 暂存工作区（临时保存修改）

```bash
# 暂存当前修改
git stash

# 恢复暂存的修改
git stash pop

# 查看暂存列表
git stash list
```

## 常见场景

### 改了文件想撤销
```bash
# 撤销未暂存的修改
git checkout -- <file>

# 撤销已暂存的修改（从暂存区移除）
git reset HEAD <file>
```

### 提交信息写错了（还没推送）
```bash
git commit --amend -m "正确的提交信息"
```

### 查看某个文件的修改内容
```bash
git diff <file>              # 查看未暂存的修改
git diff --staged            # 查看已暂存的修改
```

## 注意事项

- `models/` 和 `uwb-reference/` 已被 `.gitignore` 排除，不会被推送
- 每次 `git commit` 后会自动推送到 GitHub，无需手动 `git push`
- 如果自动推送失败，会提示错误信息，可手动执行 `git push origin main`
