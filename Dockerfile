FROM python:3.10-slim
RUN mkdir -p /pip_cache
ENV PIP_CACHE_DIR=/pip_cache
WORKDIR /pid
ADD . .
#COPY requirements.txt /tmp/requirements.txt

RUN pip --default-timeout=3000 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
CMD ["python", "./main.py"]
