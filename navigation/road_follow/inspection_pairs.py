"""Optional, explicitly supplied face-to-knife correspondence."""
FACE_KNIFE_PAIRS = {8: 7, 3: 5, 10: 10, 6: 1, 9: 9, 5: 2, 4: 3, 2: 6, 1: 8, 7: 4}
PAIR_TIMEOUT_S = 15.0
PAIR_SPEECH_DELAY_S = 3.0


def paired_knife_identity(face_identity):
    for person, knife in FACE_KNIFE_PAIRS.items():
        if face_identity == f"suspect_{person:02d}":
            return f"knife_{knife:02d}"
    return None
