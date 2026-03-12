FROM espressif/idf:v5.3

ARG ESP_ZB_SDK_REF=main
ENV IDF_PYTHON=/opt/esp/python_env/idf5.3_py3.10_env/bin/python
ENV PATH=/opt/esp/python_env/idf5.3_py3.10_env/bin:${PATH}

RUN git clone --depth 1 --branch ${ESP_ZB_SDK_REF} https://github.com/espressif/esp-zigbee-sdk /opt/esp-zigbee-sdk \
    && ${IDF_PYTHON} -m pip install --no-cache-dir zigpy

ENV ESP_ZB_SDK_PATH=/opt/esp-zigbee-sdk
