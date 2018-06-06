FROM ros:kinetic
WORKDIR /Engagement
ADD . /Engagement
RUN /bin/bash -c 'git clone https://github.com/mingfeisun/markov_decision_making.git'
