Fonctionne sur carte STM32F072.

Pour faire fonctionner ce projet, il faut premi�rement t�l�charger les logiciels suivant :
- La version actuelle de Factory I/O 2.4.5 (prendre l'essaie gratuit) : https://realgames.b-cdn.net/fio/factoryio-installer-latest.exe
- La version 3.7.9 de Python : https://www.python.org/ftp/python/3.7.9/python-3.7.9-amd64.exe
- Installation de l'environnement STM32 : se r�f�rer au tuto de Pomad : http://www.pomad.fr/node/2

Installation de Python :
Une fois les logiciels pr�c�dents install�s, ouvrez un terminal de commande (cmd).
Tapez "pip install pythonnet" puis attendez.
Ensuite, tappez "pip install pyserial" et attendez la fin de l'installation.
C'est termin�.

Mise en place de FactoryIO :
Allez dans les fichier du logiciel. Dans le dossier "scene", ajoutez-y la scene "Paletiseur_1_4_OSTR".

Ouvrez IDLE Python pour y ouvrir le fichier "bridge", Atollic pour y ouvrir le projet STM32 puis 
FactotyIO pour la scene du paletiseur "Paletiseur_1_4_OSTR".
Lancez en premier le bridge, en second la scene du paletiseur et pour finir, d�buguez le projet STM32.

Appuyez sur le bouton "Utilisateur" (en bleu) de la carte STM pour lancer le paletiseur.