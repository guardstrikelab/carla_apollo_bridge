FROM auroai/cyber:v5.0.0
# cyber / bridge requirements
RUN apt update && \
    apt install -y python-pip && \
    pip install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip install \
    numpy \
    opencv-python==4.2.0.32 \
    protobuf==3.17.3 \
    pygame \
    pyproj \
    pyyaml \
    transforms3d\
    carla==0.9.13
RUN echo "source /apollo/cyber/setup.bash" >> ~/.bashrc && \
    echo "export CARLA_PYTHON_ROOT=/apollo/cyber/carla_bridge/carla_python" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:/apollo/py_proto" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:/apollo/cyber" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:\$CARLA_PYTHON_ROOT/carla" >> ~/.bashrc && \
    echo "" >> ~/.bashrc
COPY setup.bash /apollo/cyber/

RUN rm -rf /root/.cache/*
# RUN /bin/bash -c "source /root/.bashrc"
# RUN /apollo/apollo.sh build_cyber opt

CMD /bin/bash -c "source /root/.bashrc" && sleep infinity
