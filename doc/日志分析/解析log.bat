@echo off
rem 双击本文件 -> 弹出选择框选一个 log 的 txt -> 在同目录生成同名 xlsx
start "" pythonw "%~dp0parse_logs.py"
