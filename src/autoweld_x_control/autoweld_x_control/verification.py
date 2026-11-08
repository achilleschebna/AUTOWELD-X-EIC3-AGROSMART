"""Verification des trajectoires avant envoi aux robots (AUTOWELD-X).

Equipe AGROSMART-X - EIC 3.0
Responsable : PIERRE Hervens (Programmation Python / logique de controle)

Module en Python pur (sans ROS 2) : il se teste sans simulateur.
Les limites viennent de robots/bras_3ddl.xacro.
"""

LIMITES = {
    1: (-3.14, 3.14),   # base (rad)
    2: (-1.57, 1.57),   # epaule (rad)
    3: (-2.0, 2.0),     # coude (rad)
}
VITESSE_MAX = 1.5       # rad/s, limite "velocity" du xacro


def verifier_pose(positions):
    """Retourne la liste des erreurs d'une pose (liste vide si elle est valide)."""
    if len(positions) != 3:
        return [f'3 articulations attendues, {len(positions)} recues']
    erreurs = []
    for i, q in enumerate(positions, start=1):
        bas, haut = LIMITES[i]
        if not bas <= q <= haut:
            erreurs.append(f'joint_{i} = {q:.3f} rad hors limites [{bas}, {haut}]')
    return erreurs


def verifier_trajectoire(points, depart=None):
    """Verifie une trajectoire [(positions, temps_en_s), ...].

    Controle : poses dans les limites, temps strictement croissants,
    vitesse articulaire inferieure a VITESSE_MAX. `depart` est la pose
    initiale (facultative) utilisee pour verifier la vitesse du 1er segment.
    Retourne la liste des erreurs (vide si la trajectoire est valide).
    """
    if not points:
        return ['trajectoire vide']
    erreurs = []
    precedent = depart
    t_prec = 0.0
    for n, (positions, t) in enumerate(points, start=1):
        for e in verifier_pose(positions):
            erreurs.append(f'point {n} : {e}')
        if t <= t_prec:
            erreurs.append(f'point {n} : temps {t} s non croissant (precedent {t_prec} s)')
        elif precedent is not None and len(positions) == 3 and len(precedent) == 3:
            vitesse = max(abs(a - b) for a, b in zip(positions, precedent)) / (t - t_prec)
            if vitesse > VITESSE_MAX:
                erreurs.append(
                    f'point {n} : vitesse {vitesse:.2f} rad/s > {VITESSE_MAX} rad/s')
        precedent = positions
        t_prec = t
    return erreurs
