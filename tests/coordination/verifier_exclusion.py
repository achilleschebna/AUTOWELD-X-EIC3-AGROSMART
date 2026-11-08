"""Verifie dans le journal que A et B ne sont jamais ensemble dans la zone."""
import os
import re
import sys

chemin = os.path.expanduser('~/autoweld_ws/journaux/coordination.log')
lignes = open(chemin).read().splitlines()

debuts = [i for i, l in enumerate(lignes) if 'Debut du test' in l]
if not debuts:
    print('ECHEC : aucun test trouve dans le journal')
    sys.exit(1)
lignes = lignes[debuts[-1]:]

dedans = set()
attentes = 0
entrees = {'A': 0, 'B': 0}
erreurs = []

for l in lignes:
    if 'ATTEND' in l:
        attentes += 1
    m = re.search(r'\[(A|B)\] (ENTRE dans la zone|LIBERE la zone)', l)
    if not m:
        continue
    robot, action = m.groups()
    if action.startswith('ENTRE'):
        if dedans:
            erreurs.append(f'{robot} entre alors que {sorted(dedans)} est dedans')
        dedans.add(robot)
        entrees[robot] += 1
    else:
        dedans.discard(robot)

print(f'Entrees dans la zone : A={entrees["A"]}, B={entrees["B"]}')
print(f'Attentes observees   : {attentes}')
if dedans:
    erreurs.append(f'zone jamais liberee par {sorted(dedans)}')
if erreurs:
    for e in erreurs:
        print('ECHEC :', e)
    sys.exit(1)
print('RESULTAT : SUCCES - exclusion mutuelle respectee')
