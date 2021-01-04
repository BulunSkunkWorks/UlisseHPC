echo "Launching Ulisse HPC..."
sudo mpiexec --allow-run-as-root -hostfile hostfile -np 6 python UlisseHPC_v1.0.py
