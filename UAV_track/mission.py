#import pandas as pd
# import time
#
# data1 = []
# column_name = ['a', 'b']
# ct = time.strftime('%Y-%m-%d-%H%M%S', time.localtime(time.time()))
# path = ct + str('.csv')
# log_file = pd.DataFrame(data1, columns=column_name)
#
# cnt = 0
# while cnt<10:
#     data1=[[cnt, cnt+1]]
#     frame = pd.DataFrame(data1, columns=column_name)
#     log_file = log_file.append(frame)
#     log_file.to_csv(path, index=None)
#     cnt = cnt + 1
#     time.sleep(1)
# # data = pd.read_csv(path)
#
#
# # data.to_csv(path)
# # log_file.to_csv(path, index=None)
import os
import re
import sys
def sort_key(s):
    if s:
        try:
            c = re.findall('\d+$', s)[0]
        except:
            c = -1
        return int(c)
def renameall():
	fileList = os.listdir(r"C:\\Users\\张涛\\Desktop\\1")		#待修改文件夹
	newFileList = []
	for fileName in fileList:
		var = os.path.splitext(fileName)[0]
		newFileList.append(var)
	newFileList.sort(key=sort_key, reverse=False)
	print("修改前："+str(fileList))		#输出文件夹中包含的文件
	currentpath = os.getcwd()		#得到进程当前工作目录
	os.chdir(r"C:\\Users\\张涛\\Desktop\\1")		#将当前工作目录修改为待修改文件夹的位置
	num=1		#名称变量
	for fileName in newFileList:		#遍历文件夹中所有文件
		os.rename(fileName + '.pdf',(str(num)+'.'+'pdf'))		#文件重新命名
		num = num+2		#改变编号，继续下一项
	print("---------------------------------------------------")
	os.chdir(currentpath)		#改回程序运行前的工作目录
	sys.stdin.flush()		#刷新
	print("修改后："+str(os.listdir(r"C:\\Users\\张涛\\Desktop\\1")))		#输出修改后文件夹中包含的文件
renameall()
