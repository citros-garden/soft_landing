FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **

# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog
RUN pip install --no-cache-dir --upgrade pip \
  && pip install --no-cache-dir  citros
RUN pip install pyvectorguidance setuptools==58.2.0
RUN apt update && apt-get install -y ros-humble-rosbridge-suite 

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN apt update && apt-get install -y ros-humble-rosbridge-suite 
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc