
import subprocess
import os

def run_script(script_name, log_file):
    """Start a Python script as a separate background process."""
    log_path = f"{log_file}.log"
    with open(log_path, 'ab') as file:  # Open log file in append mode
        try:
            # Start the process and detach it from the parent process group
            process = subprocess.Popen(
                ['python3', script_name],
                stdout=file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setpgrp  # Detach process from the parent's group
            )
            return process
        except Exception as e:
            # If the process fails to start, log the exception
            print(f"Failed to start {script_name}: {str(e)}")

def main():
    scripts = [
        './package_projet/serveur.py',
        './package_projet/capteur_presence.py',
        './package_projet/publisher_i2c_rand.py',
        './package_projet/site.py',
        './package_projet/client.py',
        './package_projet/suscriber_basededonne.py',
        './package_projet/suscriber_i2c.py',
        './package_projet/porte.py'
    ]

    processes = {}
    for script in scripts:
        print(f"Starting {script}...")
        process = run_script(script, script.split('.')[0])
        if process:
            processes[script] = process
            print(f"{script} is running in the background with PID {process.pid}")
        else:
            print(f"Failed to start {script}.")

    # Optionally, wait for all processes to complete
    for script, process in processes.items():
        process.wait()
        print(f"{script} finished with exit code {process.returncode}")

if __name__ == '__main__':
    main()

