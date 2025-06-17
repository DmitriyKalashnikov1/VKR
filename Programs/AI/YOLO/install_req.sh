echo -n 'Install deps for RPI or PC [0 -- RPI, other -- PC]: '
read -r mode
if [ "$mode" -eq 0 ]; then
  echo -n 'Installing for RPI'
  pip --timeout=1000 install ultralytics[export]
  pip --timeout=1000 install opencv-python
else
  pip --timeout=1000 install roboflow
  #pip --timeout=1000 install "jax[cuda12_pip]==0.4.23" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html
  pip --timeout=1000 install -r req_pc.txt
fi