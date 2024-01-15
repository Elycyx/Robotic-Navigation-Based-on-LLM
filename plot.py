import csv
import matplotlib.pyplot as plt

# 设置图形的风格（可以通过plt.style.available查看所有支持的风格）
plt.style.use('ggplot')
path = 'plots/'

def plot_csv(path, filename):
    """绘制csv文件中的数据
    """
    csv_path = path + filename + '.csv'
    x = []  # 存储横坐标数据
    y = []  # 存储纵坐标数据
    with open(csv_path, 'r') as file:
        reader = csv.reader(file)
        data = list(reader)
        x = data[0]  # 第一行作为横坐标
        y = list(map(float, data[1]))  # 第二行作为纵坐标，并转换为浮点数
    # 创建柱状图，设置柱的颜色和边缘颜色
    plt.bar(x, y, color='skyblue', edgecolor='black')
    # 添加数值标签到柱子上
    for i in range(len(x)):
        plt.text(i, y[i] + 0.05, '%.2f' % y[i], ha='center', va='bottom')
    # 设置背景色
    plt.gca().set_facecolor('white')
    # 设置标题和标签
    plt.title(filename, fontsize=14)
    plt.xlabel('models', fontsize=12)
    plt.ylabel('', fontsize=12)
    # 调整坐标轴刻度
    plt.xticks(fontsize=10, rotation=45)
    plt.yticks(fontsize=10)
    # 添加网格线
    plt.grid(True, linestyle='--', linewidth=0.5, color='grey', alpha=0.7)
    plt.savefig(path + filename + '.png')  # 保存图形
    # 显示图形
    plt.tight_layout()  # 自动调整子图参数，使之填充整个图像区域。
    plt.show()
    plt.clf()  # 清除图形



plot_csv(path, 'PL')
plot_csv(path, 'NE')
plot_csv(path, 'SR')
plot_csv(path, 'time')
plot_csv(path, 'MTR')

