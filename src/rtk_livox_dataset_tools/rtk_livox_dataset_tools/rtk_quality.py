HIGH_QUALITY = "high"
MEDIUM_QUALITY = "medium"
LOW_QUALITY = "low"


def decode_navpvt_flags(flags):
    return {
        "gnss_fix_ok": bool(flags & 1),
        "diff_soln": bool(flags & 2),
        "carrier_float": bool(flags & 64),
        "carrier_fixed": bool(flags & 128),
    }


def classify_rtk_quality(fix_type, flags, h_acc_mm, v_acc_mm, s_acc_mm_s):
    if fix_type == 3 and flags == 131:
        return HIGH_QUALITY
    if fix_type == 3 and flags == 67:
        return MEDIUM_QUALITY
    return LOW_QUALITY


def quality_rank(quality):
    if quality == HIGH_QUALITY:
        return 2
    if quality == MEDIUM_QUALITY:
        return 1
    return 0
