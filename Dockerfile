FROM jupyter/datascience-notebook

RUN pip install python-lsp-server[all] jupyter-lsp  jupyterlab_vim pystan pymc3 arviz && \
    conda install -c conda-forge nodejs && \
    jupyter labextension install @jupyter-widgets/jupyterlab-manager jupyter-matplotlib


