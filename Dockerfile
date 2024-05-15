FROM python:3.8
LABEL authors="meohong"

WORKDIR /app
COPY . /app

# Environment Setting
ENV TZ=Asia/Seoul
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV PATH /usr/local/bin:$PATH
ENV LANG C.UTF-8

# Python 3.8 pip install
RUN python3.8 -m pip install --upgrade pip

# REQUIREMENTS.txt install
RUN apt update -y
RUN pip3 install --no-cache-dir -r ./requirements.txt

# Set 'python' alias python3.8
RUN echo 'alias python=python3.8' >> .bashrc
RUN echo 'which python3.8' >> .bashrc
# Setting directory


ENTRYPOINT ["top", "-b"]