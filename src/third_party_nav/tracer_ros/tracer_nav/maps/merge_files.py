#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse
from pathlib import Path

def merge_files_to_single_file(source_folder, output_file, file_extensions=None, exclude_extensions=None):
    """
    将源文件夹下的所有文件内容合并到一个输出文件中
    
    Args:
        source_folder (str): 源文件夹路径
        output_file (str): 输出文件路径
        file_extensions (list): 要包含的文件扩展名列表，如 ['.txt', '.py']，None表示所有文件
        exclude_extensions (list): 要排除的文件扩展名列表
    """
    
    # 检查源文件夹是否存在
    if not os.path.exists(source_folder):
        print(f"错误：源文件夹 '{source_folder}' 不存在")
        return False
    
    # 获取所有文件
    all_files = []
    for root, dirs, files in os.walk(source_folder):
        for file in files:
            file_path = os.path.join(root, file)
            
            # 检查文件扩展名
            if file_extensions:
                file_ext = os.path.splitext(file)[1].lower()
                if file_ext not in file_extensions:
                    continue
            
            # 检查是否要排除
            if exclude_extensions:
                file_ext = os.path.splitext(file)[1].lower()
                if file_ext in exclude_extensions:
                    continue
            
            all_files.append(file_path)
    
    if not all_files:
        print("警告：在指定文件夹中没有找到任何文件")
        return False
    
    # 按文件名排序
    all_files.sort()
    
    try:
        with open(output_file, 'w', encoding='utf-8') as out_f:
            # 写入文件头
            out_f.write("=" * 80 + "\n")
            out_f.write(f"文件合并结果 - 源文件夹: {os.path.abspath(source_folder)}\n")
            out_f.write(f"生成时间: {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            out_f.write(f"文件总数: {len(all_files)}\n")
            out_f.write("=" * 80 + "\n\n")
            
            # 逐个处理文件
            for i, file_path in enumerate(all_files, 1):
                # 获取相对路径
                rel_path = os.path.relpath(file_path, source_folder)
                file_name = os.path.basename(file_path)
                
                try:
                    # 读取文件内容
                    with open(file_path, 'r', encoding='utf-8') as in_f:
                        content = in_f.read()
                    
                    # 写入文件分隔符和信息
                    out_f.write("\n" + "=" * 80 + "\n")
                    out_f.write(f"【文件 {i}/{len(all_files)}】\n")
                    out_f.write(f"文件名: {file_name}\n")
                    out_f.write(f"相对路径: {rel_path}\n")
                    out_f.write(f"绝对路径: {os.path.abspath(file_path)}\n")
                    out_f.write(f"文件大小: {os.path.getsize(file_path)} 字节\n")
                    out_f.write("=" * 80 + "\n\n")
                    
                    # 写入文件内容
                    out_f.write(content)
                    
                    # 如果内容没有以换行结束，添加一个换行
                    if not content.endswith('\n'):
                        out_f.write('\n')
                    
                except UnicodeDecodeError:
                    # 如果文件不是文本文件，写入提示信息
                    out_f.write("\n" + "=" * 80 + "\n")
                    out_f.write(f"【文件 {i}/{len(all_files)} - 二进制文件】\n")
                    out_f.write(f"文件名: {file_name}\n")
                    out_f.write(f"相对路径: {rel_path}\n")
                    out_f.write("=" * 80 + "\n")
                    out_f.write("[此文件为二进制文件，无法显示内容]\n\n")
                    
                except Exception as e:
                    out_f.write("\n" + "=" * 80 + "\n")
                    out_f.write(f"【文件 {i}/{len(all_files)} - 读取错误】\n")
                    out_f.write(f"文件名: {file_name}\n")
                    out_f.write(f"错误信息: {str(e)}\n")
                    out_f.write("=" * 80 + "\n\n")
        
        print(f"成功！所有文件已合并到: {os.path.abspath(output_file)}")
        print(f"共处理了 {len(all_files)} 个文件")
        return True
        
    except Exception as e:
        print(f"写入输出文件时发生错误: {str(e)}")
        return False

def main():
    # 设置命令行参数解析
    parser = argparse.ArgumentParser(description='将文件夹下的所有文件内容合并到一个文件中')
    parser.add_argument('source_folder', help='/home/songfei/catkin_ws/src/tracer_ros/tracer_nav/param')
    parser.add_argument('-o', '--output', default='merged_files.txt', 
                       help='输出文件路径 (默认: merged_files.txt)')
    parser.add_argument('-e', '--extensions', nargs='+', 
                       help='要包含的文件扩展名，例如: -e .txt .py .md')
    parser.add_argument('-x', '--exclude', nargs='+', 
                       help='要排除的文件扩展名，例如: -x .exe .bin .jpg')
    
    args = parser.parse_args()
    
    # 处理文件扩展名参数
    file_extensions = None
    if args.extensions:
        file_extensions = [ext.lower() if ext.startswith('.') else f'.{ext.lower()}' 
                          for ext in args.extensions]
    
    exclude_extensions = None
    if args.exclude:
        exclude_extensions = [ext.lower() if ext.startswith('.') else f'.{ext.lower()}' 
                             for ext in args.exclude]
    
    # 执行文件合并
    merge_files_to_single_file(args.source_folder, args.output, 
                              file_extensions, exclude_extensions)

if __name__ == "__main__":
    # 如果没有命令行参数，使用交互模式
    import sys
    if len(sys.argv) == 1:
        print("=" * 60)
        print("文件内容合并工具")
        print("=" * 60)
        
        source = input("请输入源文件夹路径: ").strip()
        output = input("请输入输出文件路径 (直接回车使用 merged_files.txt): ").strip()
        if not output:
            output = "merged_files.txt"
        
        exts = input("请输入要包含的文件扩展名 (用空格分隔，直接回车包含所有文件): ").strip()
        file_extensions = exts.split() if exts else None
        
        if file_extensions:
            file_extensions = [ext if ext.startswith('.') else f'.{ext}' for ext in file_extensions]
        
        merge_files_to_single_file(source, output, file_extensions)
    else:
        main()