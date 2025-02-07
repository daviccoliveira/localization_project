import pandas as pd
import matplotlib.pyplot as plt

# Carregando os dados do Ground Truth e da Odometry
gt_file = 'ground_truth1.csv'  # Substitua com o caminho real do arquivo
odom_file = 'odometry.1csv'    # Substitua com o caminho real do arquivo

gt_data = pd.read_csv(gt_file)
odom_data = pd.read_csv(odom_file)

# Exibindo as colunas para verificar
print("Colunas do Ground Truth:", gt_data.columns)
print("Colunas da Odometria:", odom_data.columns)

# Garantindo que as colunas que você quer acessar existem
if '%time' in gt_data.columns and 'field.pose2.position.x' in gt_data.columns and 'field.pose2.position.y' in gt_data.columns:
    gt_x = gt_data['field.pose2.position.x'].values  # Convertendo para numpy
    gt_y = gt_data['field.pose2.position.y'].values  # Convertendo para numpy
else:
    raise ValueError("Colunas necessárias não encontradas no arquivo Ground Truth")

if '%time' in odom_data.columns and 'field.pose.pose.position.x' in odom_data.columns and 'field.pose.pose.position.y' in odom_data.columns:
    odom_x = odom_data['field.pose.pose.position.x'].values  # Convertendo para numpy
    odom_y = odom_data['field.pose.pose.position.y'].values  # Convertendo para numpy
else:
    raise ValueError("Colunas necessárias não encontradas no arquivo Odometry")

# Compensando o offset entre a Odometry e o Ground Truth
# A posição do Ground Truth inicial é (x=4.64, y=3)
gt_offset_x = 4.64
gt_offset_y = 3

# Ajustando a odometria somando o offset
odom_x_adjusted = odom_x + gt_offset_x
odom_y_adjusted = odom_y + gt_offset_y

# Plotando os dados
plt.figure(figsize=(10, 6))

# Plotando o Ground Truth
plt.plot(gt_x, gt_y, label="Ground Truth", color="blue", linestyle="solid")

# Plotando a Odometry ajustada
plt.plot(odom_x_adjusted, odom_y_adjusted, label="Odometry Ajustada", color="red", linestyle="dashed")

# Adicionando rótulos e título
plt.xlabel('Posição X')
plt.ylabel('Posição Y')
plt.title('Comparação entre Ground Truth e Odometry (Ajustada)')

# Exibindo a legenda
plt.legend()

# Exibindo o gráfico
plt.grid(True)
plt.show()
