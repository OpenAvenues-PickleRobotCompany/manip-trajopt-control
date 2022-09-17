echo "Setting git hooks"
cp -r hooks .git/


echo "Installing pip-tools"
source env/bin/activate
python -m pip install pip-tools

echo "Installing Python packages"
pip-compile > requirements.txt
pip-sync

echo "Setup finished"