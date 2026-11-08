"""Detection de collision entre les bras A et B (modele geometrique simple)."""
import math

# Dimensions tirees du xacro (m)
H_EPAULE = 0.40      # hauteur de l'epaule au-dessus du sol
L_BRAS = 0.40        # epaule -> coude
L_AVANT = 0.35       # coude -> poignet
L_TORCHE = 0.12      # poignet -> pointe de la torche
RAYON_COLONNE = 0.08
RAYON_BRAS = 0.05
RAYON_AVANT = 0.04

MARGE = 0.10         # distance de securite minimale entre surfaces (m)


def points_bras(base_x, pos):
    """Retourne (base, epaule, coude, pointe) en coordonnees monde."""
    t1, t2, t3 = pos
    c, s = math.cos(t1), math.sin(t1)

    def monde(h, z):
        return (base_x + h * c, h * s, z)

    base = (base_x, 0.0, 0.0)
    epaule = (base_x, 0.0, H_EPAULE)
    coude = monde(L_BRAS * math.sin(t2), H_EPAULE + L_BRAS * math.cos(t2))
    a = t2 + t3
    h_pointe = L_BRAS * math.sin(t2) + (L_AVANT + L_TORCHE) * math.sin(a)
    z_pointe = H_EPAULE + L_BRAS * math.cos(t2) + (L_AVANT + L_TORCHE) * math.cos(a)
    pointe = monde(h_pointe, z_pointe)
    return base, epaule, coude, pointe


def segments_bras(base_x, pos):
    """Liste de (point1, point2, rayon) pour chaque element du bras."""
    b, e, c, p = points_bras(base_x, pos)
    return [(b, e, RAYON_COLONNE), (e, c, RAYON_BRAS), (c, p, RAYON_AVANT)]


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def distance_segments(p1, q1, p2, q2):
    """Distance minimale entre deux segments [p1,q1] et [p2,q2] (3D)."""
    d1, d2, r = _sub(q1, p1), _sub(q2, p2), _sub(p1, p2)
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    eps = 1e-12
    if a <= eps and e <= eps:
        s = t = 0.0
    elif a <= eps:
        s, t = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = _dot(d1, r)
        if e <= eps:
            t, s = 0.0, max(0.0, min(1.0, -c / a))
        else:
            b = _dot(d1, d2)
            den = a * e - b * b
            s = max(0.0, min(1.0, (b * f - c * e) / den)) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, max(0.0, min(1.0, (b - c) / a))
    pa = tuple(p + s * d for p, d in zip(p1, d1))
    pb = tuple(p + t * d for p, d in zip(p2, d2))
    return math.dist(pa, pb)


def degagement(pos_a, pos_b, base_a=-0.8, base_b=0.8):
    """Plus petite distance entre les SURFACES des deux bras (m)."""
    mini = float('inf')
    for (p1, q1, r1) in segments_bras(base_a, pos_a):
        for (p2, q2, r2) in segments_bras(base_b, pos_b):
            d = distance_segments(p1, q1, p2, q2) - r1 - r2
            mini = min(mini, d)
    return mini


def en_collision(pos_a, pos_b, marge=MARGE):
    return degagement(pos_a, pos_b) < marge
