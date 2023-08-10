# syntax=docker/dockerfile:1

FROM python:3.10

WORKDIR /app

COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt
RUN pip3 install --trusted-host www.open3d.org -f http://www.open3d.org/docs/latest/getting_started.html open3d
RUN pip3 install open3d-cpu
# RUN apt-get update && apt-get install -y ffmpeg libsm6 libxext6
RUN apt-get update && apt-get install -y python3-opencv
RUN pip3 install opencv-python
COPY . .

CMD ["python3", "-m", "main", "run"]