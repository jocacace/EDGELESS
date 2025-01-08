#Base image used to create the new image
FROM ubuntu:jammy

# User and group ids of in the host machine
# To check their value use these commands
# $ id -u  (for the UID)
# $ id -g  (for the GUI)
ARG UID=1000
ARG GID=1000

RUN locale
RUN apt-get update && apt-get install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
 
RUN apt-get install software-properties-common -y
RUN add-apt-repository universe

RUN apt update && \
    apt install -y sudo && \
    addgroup --gid $GID user && \
    adduser --uid $UID --gid $GID --disabled-password --gecos "" user && \
    echo 'user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

RUN apt-get install curl -y
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -y | sh

RUN apt install gcc libssl-dev pkg-config protobuf-compiler make g++ -y

USER user
CMD ["/bin/bash"]




