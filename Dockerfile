FROM nvcr.io/nvidia/isaac-lab:2.0.0
WORKDIR /isaac-sim
COPY additional_requirements.txt /isaac-sim
RUN ./python.sh -m pip install -r additional_requirements.txt

ENTRYPOINT ["/bin/bash"]
