from PIL import Image, ImageDraw
import random
import math
import os
import shutil

# Deleta a pasta 'img' (se existir)
if os.path.exists('img'):
    shutil.rmtree('img')

# Cria a pasta 'img'
os.mkdir('img')

# Define o tamanho da imagem
w = 1080
h = 1920

# Inicializar a imagem
image = Image.new("RGB", (w, h), (255, 240, 225))  # Cor de fundo rosa (RGB)
draw = ImageDraw.Draw(image)

# Gerar vértices
vertices = []
raio = 20

while len(vertices) < 100:
    x = random.randint(raio, w - raio)  # Ajuste dos limites de x
    y = random.randint(raio, h - raio)  # Ajuste dos limites de y

    valid = True
    for v in vertices:
        x1, y1 = v
        distancia = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
        if distancia < 100:
            valid = False
            break

    if valid:
        vertices.append((x, y))

# Gerar arestas
arestas = []
for i in range(len(vertices)):
    x1, y1 = vertices[i]

    for j in range(i + 1, len(vertices)):
        x2, y2 = vertices[j]
        distancia = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if distancia < 200:
            arestas.append((i, j, distancia))

# Desenhar vértices
for vertex in vertices:
    x, y = vertex
    draw.ellipse([(x - raio, y - raio), (x + raio, y + raio)], fill=(255, 150, 200))  # Cor rosa chiclete (RGB)

# Desenhar arestas
for aresta in arestas:
    i, j, _ = aresta
    x1, y1 = vertices[i]
    x2, y2 = vertices[j]
    draw.line([(x1, y1), (x2, y2)], fill=(255, 150, 200), width=6)  # Cor rosa chiclete (RGB)

# Salvar a imagem do grafo original
image.save("grafo.png")

# Algoritmo de Kruskal
arestas.sort(key=lambda x: x[2])
agm = []
parent = [i for i in range(len(vertices))]

def find(i):
    if parent[i] != i:
        parent[i] = find(parent[i])
    return parent[i]

def union(i, j):
    parent[find(i)] = find(j)

frame = 0  # Inicializar a variável frame

for aresta in arestas:
    i, j, d = aresta
    if find(i) != find(j):
        union(i, j)
        agm.append(aresta)
        x1, y1 = vertices[i]
        x2, y2 = vertices[j]
        draw.line([(x1, y1), (x2, y2)], fill=(127, 96, 101), width=6)  # Cor rosa (RGB)

        # Salvar o frame
        image.save(f"img/frame_{frame:04}.png")
        frame += 1

# Salvar a imagem final com a AGM
image.save("grafo_agm.png")

import cv2
import os
from PIL import Image

# Obter a lista de frames
frame_files = sorted(os.listdir("img"))

# Definir a duração desejada do vídeo (em segundos)
desired_duration = 30

# Definir a taxa de quadros por segundo (FPS)
fps = len(frame_files) / desired_duration

# Definir o tamanho do vídeo
width, height = (1080, 1920)

# Inicializar o objeto VideoWriter
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
video = cv2.VideoWriter("output.mp4", fourcc, fps, (width, height))

# Adicionar cada frame ao vídeo
for frame_file in frame_files:
    frame_path = os.path.join("img", frame_file)
    frame = cv2.imread(frame_path)
    video.write(frame)

# Carregar o último frame (grafo_agm.png)
last_frame_path = "grafo_agm.png"
last_frame = cv2.imread(last_frame_path)

# Adicionar o último frame duplicado ao vídeo
total_frames = int(desired_duration * fps)
extra_frames = total_frames - len(frame_files)

if extra_frames > 0:
    for _ in range(extra_frames):
        video.write(last_frame)

# Liberar recursos
video.release()

import cv2
import os
from PIL import Image
from moviepy.editor import VideoFileClip, AudioFileClip

# Definir o caminho do vídeo gerado
video_path = "output.mp4"

# Definir o caminho do arquivo de áudio
audio_path = "687439__josefpres__piano-loops-102-octave-short-loop-120-bpm.wav"

# Definir o caminho de saída do vídeo com áudio
output_path = "output_with_audio.mp4"

# Carregar o vídeo gerado
video = VideoFileClip(video_path)

# Carregar o arquivo de áudio
audio = AudioFileClip(audio_path)

# Sincronizar a duração do áudio com a duração do vídeo
audio = audio.set_duration(video.duration)

# Adicionar o áudio ao vídeo
video_with_audio = video.set_audio(audio)

# Salvar o vídeo resultante com áudio
video_with_audio.write_videofile(output_path, codec="libx264", audio_codec="aac")

# Fechar os objetos de vídeo e áudio
video.close()
audio.close()