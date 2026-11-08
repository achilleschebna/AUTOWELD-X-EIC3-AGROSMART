"""Tests de la detection de collision entre les bras A et B."""
import os
import sys

sys.path.insert(0, os.path.expanduser(
    '~/AUTOWELD-X-EIC3-AGROSMART/src/autoweld_x_control/autoweld_x_control'))
from collision import degagement, en_collision, distance_segments  # noqa: E402

DEPART = [0.0, 0.0, 0.0]
APPROCHE = [0.0, 0.6, 0.3]
TRAVAIL_1 = [0.0, 1.2, 0.4]


def miroir(p):
    return [p[0], -p[1], -p[2]]


resultats = []


def cas(nom, condition, detail):
    ok = bool(condition)
    resultats.append(ok)
    print(f'[{"OK" if ok else "ECHEC"}] {nom} : {detail}')


d = distance_segments((0, 0, 0), (1, 0, 0), (0.5, -1, 1), (0.5, 1, 1))
cas('1. distance entre deux segments croises', abs(d - 1.0) < 1e-6, f'{d:.3f} m (attendu 1.000)')

d = degagement(DEPART, DEPART)
cas('2. deux robots au repos : pas de collision', not en_collision(DEPART, DEPART), f'degagement = {d:.2f} m')

d = degagement(APPROCHE, miroir(APPROCHE))
cas('3. les deux a "approche" : distance suffisante', not en_collision(APPROCHE, miroir(APPROCHE)), f'degagement = {d:.2f} m')

d = degagement(TRAVAIL_1, DEPART)
cas('4. A au travail, B au repos : pas de collision', not en_collision(TRAVAIL_1, DEPART), f'degagement = {d:.2f} m')

d = degagement(TRAVAIL_1, miroir(TRAVAIL_1))
cas('5. A et B ensemble au point de travail : COLLISION detectee', en_collision(TRAVAIL_1, miroir(TRAVAIL_1)), f'degagement = {d:+.2f} m (negatif = chevauchement)')

print()
print(f'{sum(resultats)}/{len(resultats)} cas reussis')
if not all(resultats):
    print('RESULTAT : ECHEC')
    sys.exit(1)
print('RESULTAT : SUCCES - detection de collision validee')
