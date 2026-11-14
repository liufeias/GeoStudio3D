#include "geo_kernel/exact_predicates.hpp"

#include <atomic>
#include <cmath>
#include <mutex>

namespace geo_kernel {

namespace {

std::once_flag g_init_once_flag;

double g_splitter = 0.0;
double g_epsilon = 0.0;
double g_resulterrbound = 0.0;
double g_ccwerrboundA = 0.0;
double g_ccwerrboundB = 0.0;
double g_ccwerrboundC = 0.0;
double g_iccerrboundA = 0.0;
double g_iccerrboundB = 0.0;
double g_iccerrboundC = 0.0;

inline double Absolute(double a) noexcept {
    return a >= 0.0 ? a : -a;
}

inline void FastTwoSum(double a, double b, double& x, double& y) noexcept {
    x = a + b;
    const double bvirt = x - a;
    y = b - bvirt;
}

inline void TwoSum(double a, double b, double& x, double& y) noexcept {
    x = a + b;
    const double bvirt = x - a;
    const double avirt = x - bvirt;
    const double bround = b - bvirt;
    const double around = a - avirt;
    y = around + bround;
}

inline void TwoDiff(double a, double b, double& x, double& y) noexcept {
    x = a - b;
    const double bvirt = a - x;
    const double avirt = x + bvirt;
    const double bround = bvirt - b;
    const double around = a - avirt;
    y = around + bround;
}

inline void Split(double a, double& ahi, double& alo) noexcept {
    const double c = g_splitter * a;
    const double abig = c - a;
    ahi = c - abig;
    alo = a - ahi;
}

inline void TwoProduct(double a, double b, double& x, double& y) noexcept {
    x = a * b;
    double ahi = 0.0, alo = 0.0, bhi = 0.0, blo = 0.0;
    Split(a, ahi, alo);
    Split(b, bhi, blo);
    const double err1 = x - (ahi * bhi);
    const double err2 = err1 - (alo * bhi);
    const double err3 = err2 - (ahi * blo);
    y = (alo * blo) - err3;
}

inline void TwoProductPresplit(double a, double b, double bhi, double blo, double& x, double& y) noexcept {
    x = a * b;
    double ahi = 0.0, alo = 0.0;
    Split(a, ahi, alo);
    const double err1 = x - (ahi * bhi);
    const double err2 = err1 - (alo * bhi);
    const double err3 = err2 - (ahi * blo);
    y = (alo * blo) - err3;
}

inline void TwoTwoSum(double a1, double a0, double b1, double b0,
                      double& x3, double& x2, double& x1, double& x0) noexcept {
    double _i = 0.0, _j = 0.0, _0 = 0.0;
    TwoSum(a0, b0, _i, x0);
    TwoSum(a1, _i, _j, _0);
    TwoSum(_0, b1, _i, x1);
    TwoSum(_j, _i, x3, x2);
}

inline void TwoTwoDiff(double a1, double a0, double b1, double b0,
                       double& x3, double& x2, double& x1, double& x0) noexcept {
    double _i = 0.0, _j = 0.0, _0 = 0.0;
    TwoDiff(a0, b0, _i, x0);
    TwoDiff(a1, _i, _j, _0);
    TwoDiff(_0, b1, _i, x1);
    TwoSum(_j, _i, x3, x2);
}

double Estimate(int elen, const double* e) noexcept {
    double q = e[0];
    for (int i = 1; i < elen; ++i) {
        q += e[i];
    }
    return q;
}

int FastExpansionSumZeroElim(int elen, const double* e, int flen, const double* f, double* h) noexcept {
    double q = 0.0;
    double qnew = 0.0;
    double hh = 0.0;
    int eindex = 0;
    int findex = 0;
    int hindex = 0;

    double enow = e[0];
    double fnow = f[0];

    if ((fnow > enow) == (fnow > -enow)) {
        q = enow;
        eindex++;
        if (eindex < elen) enow = e[eindex];
    } else {
        q = fnow;
        findex++;
        if (findex < flen) fnow = f[findex];
    }

    if ((eindex < elen) && (findex < flen)) {
        if ((fnow > enow) == (fnow > -enow)) {
            FastTwoSum(enow, q, qnew, hh);
            eindex++;
            if (eindex < elen) enow = e[eindex];
        } else {
            FastTwoSum(fnow, q, qnew, hh);
            findex++;
            if (findex < flen) fnow = f[findex];
        }
        q = qnew;
        if (hh != 0.0) {
            h[hindex++] = hh;
        }
        while ((eindex < elen) && (findex < flen)) {
            if ((fnow > enow) == (fnow > -enow)) {
                TwoSum(q, enow, qnew, hh);
                eindex++;
                if (eindex < elen) enow = e[eindex];
            } else {
                TwoSum(q, fnow, qnew, hh);
                findex++;
                if (findex < flen) fnow = f[findex];
            }
            q = qnew;
            if (hh != 0.0) {
                h[hindex++] = hh;
            }
        }
    }

    while (eindex < elen) {
        TwoSum(q, enow, qnew, hh);
        eindex++;
        if (eindex < elen) enow = e[eindex];
        q = qnew;
        if (hh != 0.0) {
            h[hindex++] = hh;
        }
    }

    while (findex < flen) {
        TwoSum(q, fnow, qnew, hh);
        findex++;
        if (findex < flen) fnow = f[findex];
        q = qnew;
        if (hh != 0.0) {
            h[hindex++] = hh;
        }
    }

    if ((q != 0.0) || (hindex == 0)) {
        h[hindex++] = q;
    }
    return hindex;
}

int ScaleExpansionZeroElim(int elen, const double* e, double b, double* h) noexcept {
    double q = 0.0;
    double sum = 0.0;
    double hh = 0.0;
    double product1 = 0.0;
    double product0 = 0.0;
    int hindex = 0;

    double bhi = 0.0, blo = 0.0;
    Split(b, bhi, blo);

    TwoProductPresplit(e[0], b, bhi, blo, q, hh);
    if (hh != 0.0) {
        h[hindex++] = hh;
    }

    for (int eindex = 1; eindex < elen; ++eindex) {
        const double enow = e[eindex];
        TwoProductPresplit(enow, b, bhi, blo, product1, product0);
        TwoSum(q, product0, sum, hh);
        if (hh != 0.0) {
            h[hindex++] = hh;
        }
        FastTwoSum(product1, sum, q, hh);
        if (hh != 0.0) {
            h[hindex++] = hh;
        }
    }

    if ((q != 0.0) || (hindex == 0)) {
        h[hindex++] = q;
    }
    return hindex;
}

double Orient2DAdapt(double ax, double ay, double bx, double by, double cx, double cy, double detsum) noexcept {
    double acx = ax - cx;
    double bcx = bx - cx;
    double acy = ay - cy;
    double bcy = by - cy;

    double detleft = 0.0, detlefttail = 0.0;
    double detright = 0.0, detrighttail = 0.0;
    TwoProduct(acx, bcy, detleft, detlefttail);
    TwoProduct(acy, bcx, detright, detrighttail);

    double B[4];
    TwoTwoDiff(detleft, detlefttail, detright, detrighttail, B[3], B[2], B[1], B[0]);

    double det = Estimate(4, B);
    double errbound = g_ccwerrboundB * detsum;
    if ((det >= errbound) || (-det >= errbound)) {
        return det;
    }

    double acxtail = 0.0, bcxtail = 0.0, acytail = 0.0, bcytail = 0.0;
    TwoDiff(ax, cx, acx, acxtail);
    TwoDiff(bx, cx, bcx, bcxtail);
    TwoDiff(ay, cy, acy, acytail);
    TwoDiff(by, cy, bcy, bcytail);

    if ((acxtail == 0.0) && (acytail == 0.0) && (bcxtail == 0.0) && (bcytail == 0.0)) {
        return det;
    }

    errbound = g_ccwerrboundC * detsum + g_resulterrbound * Absolute(det);
    det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
    if ((det >= errbound) || (-det >= errbound)) {
        return det;
    }

    double s1 = 0.0, s0 = 0.0, t1 = 0.0, t0 = 0.0;
    double u[4];
    double C1[8];
    double C2[12];
    double D[16];

    TwoProduct(acxtail, bcy, s1, s0);
    TwoProduct(acytail, bcx, t1, t0);
    TwoTwoDiff(s1, s0, t1, t0, u[3], u[2], u[1], u[0]);
    const int C1length = FastExpansionSumZeroElim(4, B, 4, u, C1);

    TwoProduct(acx, bcytail, s1, s0);
    TwoProduct(acy, bcxtail, t1, t0);
    TwoTwoDiff(s1, s0, t1, t0, u[3], u[2], u[1], u[0]);
    const int C2length = FastExpansionSumZeroElim(C1length, C1, 4, u, C2);

    TwoProduct(acxtail, bcytail, s1, s0);
    TwoProduct(acytail, bcxtail, t1, t0);
    TwoTwoDiff(s1, s0, t1, t0, u[3], u[2], u[1], u[0]);
    const int Dlength = FastExpansionSumZeroElim(C2length, C2, 4, u, D);

    return D[Dlength - 1];
}

double InCircleAdapt(double ax, double ay, double bx, double by,
                     double cx, double cy, double dx, double dy,
                     double permanent) noexcept {
    double adx = ax - dx;
    double bdx = bx - dx;
    double cdx = cx - dx;
    double ady = ay - dy;
    double bdy = by - dy;
    double cdy = cy - dy;

    double bdxcdy1 = 0.0, bdxcdy0 = 0.0, cdxbdy1 = 0.0, cdxbdy0 = 0.0;
    TwoProduct(bdx, cdy, bdxcdy1, bdxcdy0);
    TwoProduct(cdx, bdy, cdxbdy1, cdxbdy0);
    double bc[4];
    TwoTwoDiff(bdxcdy1, bdxcdy0, cdxbdy1, cdxbdy0, bc[3], bc[2], bc[1], bc[0]);

    double adet[8], bdet[8], cdet[8];
    double axbc[8], axxbc[16], aybc[8], ayybc[16];
    int axbclen = ScaleExpansionZeroElim(4, bc, adx, axbc);
    int axxbclen = ScaleExpansionZeroElim(axbclen, axbc, adx, axxbc);
    int aybclen = ScaleExpansionZeroElim(4, bc, ady, aybc);
    int ayybclen = ScaleExpansionZeroElim(aybclen, aybc, ady, ayybc);
    int alen = FastExpansionSumZeroElim(axxbclen, axxbc, ayybclen, ayybc, adet);

    double cdxady1 = 0.0, cdxady0 = 0.0, adxcdy1 = 0.0, adxcdy0 = 0.0;
    TwoProduct(cdx, ady, cdxady1, cdxady0);
    TwoProduct(adx, cdy, adxcdy1, adxcdy0);
    double ca[4];
    TwoTwoDiff(cdxady1, cdxady0, adxcdy1, adxcdy0, ca[3], ca[2], ca[1], ca[0]);

    double bxca[8], bxxca[16], byca[8], byyca[16];
    int bxcalen = ScaleExpansionZeroElim(4, ca, bdx, bxca);
    int bxxcalen = ScaleExpansionZeroElim(bxcalen, bxca, bdx, bxxca);
    int bycalen = ScaleExpansionZeroElim(4, ca, bdy, byca);
    int byycalen = ScaleExpansionZeroElim(bycalen, byca, bdy, byyca);
    int blen = FastExpansionSumZeroElim(bxxcalen, bxxca, byycalen, byyca, bdet);

    double adxbdy1 = 0.0, adxbdy0 = 0.0, bdxady1 = 0.0, bdxady0 = 0.0;
    TwoProduct(adx, bdy, adxbdy1, adxbdy0);
    TwoProduct(bdx, ady, bdxady1, bdxady0);
    double ab[4];
    TwoTwoDiff(adxbdy1, adxbdy0, bdxady1, bdxady0, ab[3], ab[2], ab[1], ab[0]);

    double cxab[8], cxxab[16], cyab[8], cyyab[16];
    int cxablen = ScaleExpansionZeroElim(4, ab, cdx, cxab);
    int cxxablen = ScaleExpansionZeroElim(cxablen, cxab, cdx, cxxab);
    int cyablen = ScaleExpansionZeroElim(4, ab, cdy, cyab);
    int cyyablen = ScaleExpansionZeroElim(cyablen, cyab, cdy, cyyab);
    int clen = FastExpansionSumZeroElim(cxxablen, cxxab, cyyablen, cyyab, cdet);

    double abdet[16];
    double fin1[1152], fin2[1152];
    int ablen = FastExpansionSumZeroElim(alen, adet, blen, bdet, abdet);
    int finlength = FastExpansionSumZeroElim(ablen, abdet, clen, cdet, fin1);

    double det = Estimate(finlength, fin1);
    double errbound = g_iccerrboundB * permanent;
    if ((det >= errbound) || (-det >= errbound)) {
        return det;
    }

    double adxtail = 0.0, bdxtail = 0.0, cdxtail = 0.0;
    double adytail = 0.0, bdytail = 0.0, cdytail = 0.0;
    TwoDiff(ax, dx, adx, adxtail);
    TwoDiff(bx, dx, bdx, bdxtail);
    TwoDiff(cx, dx, cdx, cdxtail);
    TwoDiff(ay, dy, ady, adytail);
    TwoDiff(by, dy, bdy, bdytail);
    TwoDiff(cy, dy, cdy, cdytail);

    if ((adxtail == 0.0) && (bdxtail == 0.0) && (cdxtail == 0.0) &&
        (adytail == 0.0) && (bdytail == 0.0) && (cdytail == 0.0)) {
        return det;
    }

    errbound = g_iccerrboundC * permanent + g_resulterrbound * Absolute(det);
    det += ((adx * adx + ady * ady) * ((bdx * cdytail + cdy * bdxtail) - (bdy * cdxtail + cdx * bdytail)) +
            2.0 * (adx * adxtail + ady * adytail) * (bdx * cdy - bdy * cdx)) +
           ((bdx * bdx + bdy * bdy) * ((cdx * adytail + ady * cdxtail) - (cdy * adxtail + adx * cdytail)) +
            2.0 * (bdx * bdxtail + bdy * bdytail) * (cdx * ady - cdy * adx)) +
           ((cdx * cdx + cdy * cdy) * ((adx * bdytail + bdy * adxtail) - (ady * bdxtail + bdx * adytail)) +
            2.0 * (cdx * cdxtail + cdy * cdytail) * (adx * bdy - ady * bdx));
    if ((det >= errbound) || (-det >= errbound)) {
        return det;
    }

    double* finnow = fin1;
    double* finother = fin2;
    auto swap_fin = [&]() noexcept {
        double* tmp = finnow;
        finnow = finother;
        finother = tmp;
    };

    double aa[4], bb[4], cc[4];
    int aalen = 0, bblen = 0, cclen = 0;
    if ((bdxtail != 0.0) || (bdytail != 0.0) || (cdxtail != 0.0) || (cdytail != 0.0)) {
        double adxadx1 = 0.0, adxadx0 = 0.0, adyady1 = 0.0, adyady0 = 0.0;
        TwoProduct(adx, adx, adxadx1, adxadx0);
        TwoProduct(ady, ady, adyady1, adyady0);
        TwoTwoSum(adxadx1, adxadx0, adyady1, adyady0, aa[3], aa[2], aa[1], aa[0]);
        aalen = 4;
    }
    if ((cdxtail != 0.0) || (cdytail != 0.0) || (adxtail != 0.0) || (adytail != 0.0)) {
        double bdxbdx1 = 0.0, bdxbdx0 = 0.0, bdybdy1 = 0.0, bdybdy0 = 0.0;
        TwoProduct(bdx, bdx, bdxbdx1, bdxbdx0);
        TwoProduct(bdy, bdy, bdybdy1, bdybdy0);
        TwoTwoSum(bdxbdx1, bdxbdx0, bdybdy1, bdybdy0, bb[3], bb[2], bb[1], bb[0]);
        bblen = 4;
    }
    if ((adxtail != 0.0) || (adytail != 0.0) || (bdxtail != 0.0) || (bdytail != 0.0)) {
        double cdxcdx1 = 0.0, cdxcdx0 = 0.0, cdycdy1 = 0.0, cdycdy0 = 0.0;
        TwoProduct(cdx, cdx, cdxcdx1, cdxcdx0);
        TwoProduct(cdy, cdy, cdycdy1, cdycdy0);
        TwoTwoSum(cdxcdx1, cdxcdx0, cdycdy1, cdycdy0, cc[3], cc[2], cc[1], cc[0]);
        cclen = 4;
    }

    double temp8[8], temp16a[16], temp16b[16], temp16c[16], temp32a[32], temp32b[32], temp48[48], temp64[64];
    double u[4];
    double s1 = 0.0, s0 = 0.0, t1 = 0.0, t0 = 0.0;

    if (adxtail != 0.0) {
        int temp8len = ScaleExpansionZeroElim(4, bc, adxtail, temp8);
        int temp16alen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * adx, temp16a);
        int temp8len2 = ScaleExpansionZeroElim(4, cc, adxtail, temp8);
        int temp16blen = ScaleExpansionZeroElim(temp8len2, temp8, bdy, temp16b);
        int temp8len3 = ScaleExpansionZeroElim(4, bb, adxtail, temp8);
        int temp16clen = ScaleExpansionZeroElim(temp8len3, temp8, -cdy, temp16c);
        int temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
        int temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
        finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
        swap_fin();
    }
    if (adytail != 0.0) {
        int temp8len = ScaleExpansionZeroElim(4, bc, adytail, temp8);
        int temp16alen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * ady, temp16a);
        int temp8len2 = ScaleExpansionZeroElim(4, bb, adytail, temp8);
        int temp16blen = ScaleExpansionZeroElim(temp8len2, temp8, cdx, temp16b);
        int temp8len3 = ScaleExpansionZeroElim(4, cc, adytail, temp8);
        int temp16clen = ScaleExpansionZeroElim(temp8len3, temp8, -bdx, temp16c);
        int temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
        int temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
        finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
        swap_fin();
    }
    if (bdxtail != 0.0) {
        int temp8len = ScaleExpansionZeroElim(4, ca, bdxtail, temp8);
        int temp16alen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * bdx, temp16a);
        int temp8len2 = ScaleExpansionZeroElim(4, aa, bdxtail, temp8);
        int temp16blen = ScaleExpansionZeroElim(temp8len2, temp8, cdy, temp16b);
        int temp8len3 = ScaleExpansionZeroElim(4, cc, bdxtail, temp8);
        int temp16clen = ScaleExpansionZeroElim(temp8len3, temp8, -ady, temp16c);
        int temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
        int temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
        finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
        swap_fin();
    }
    if (bdytail != 0.0) {
        int temp8len = ScaleExpansionZeroElim(4, ca, bdytail, temp8);
        int temp16alen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * bdy, temp16a);
        int temp8len2 = ScaleExpansionZeroElim(4, cc, bdytail, temp8);
        int temp16blen = ScaleExpansionZeroElim(temp8len2, temp8, adx, temp16b);
        int temp8len3 = ScaleExpansionZeroElim(4, aa, bdytail, temp8);
        int temp16clen = ScaleExpansionZeroElim(temp8len3, temp8, -cdx, temp16c);
        int temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
        int temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
        finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
        swap_fin();
    }
    if (cdxtail != 0.0) {
        int temp8len = ScaleExpansionZeroElim(4, ab, cdxtail, temp8);
        int temp16alen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * cdx, temp16a);
        int temp8len2 = ScaleExpansionZeroElim(4, bb, cdxtail, temp8);
        int temp16blen = ScaleExpansionZeroElim(temp8len2, temp8, ady, temp16b);
        int temp8len3 = ScaleExpansionZeroElim(4, aa, cdxtail, temp8);
        int temp16clen = ScaleExpansionZeroElim(temp8len3, temp8, -bdy, temp16c);
        int temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
        int temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
        finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
        swap_fin();
    }
    if (cdytail != 0.0) {
        int temp8len = ScaleExpansionZeroElim(4, ab, cdytail, temp8);
        int temp16alen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * cdy, temp16a);
        int temp8len2 = ScaleExpansionZeroElim(4, aa, cdytail, temp8);
        int temp16blen = ScaleExpansionZeroElim(temp8len2, temp8, bdx, temp16b);
        int temp8len3 = ScaleExpansionZeroElim(4, bb, cdytail, temp8);
        int temp16clen = ScaleExpansionZeroElim(temp8len3, temp8, -adx, temp16c);
        int temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
        int temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
        finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
        swap_fin();
    }

    if ((adxtail != 0.0) || (adytail != 0.0)) {
        int bctlen = 0, bcttlen = 0;
        double bct[8], bctt[4];
        if ((bdxtail != 0.0) || (bdytail != 0.0) || (cdxtail != 0.0) || (cdytail != 0.0)) {
            TwoProduct(bdxtail, cdy, s1, s0);
            TwoProduct(bdx, cdytail, t1, t0);
            TwoTwoSum(s1, s0, t1, t0, u[3], u[2], u[1], u[0]);
            TwoProduct(cdxtail, -bdy, s1, s0);
            TwoProduct(cdx, -bdytail, t1, t0);
            TwoTwoSum(s1, s0, t1, t0, temp8[3], temp8[2], temp8[1], temp8[0]);
            bctlen = FastExpansionSumZeroElim(4, u, 4, temp8, bct);

            TwoProduct(bdxtail, cdytail, s1, s0);
            TwoProduct(cdxtail, bdytail, t1, t0);
            TwoTwoDiff(s1, s0, t1, t0, bctt[3], bctt[2], bctt[1], bctt[0]);
            bcttlen = 4;
        } else {
            bct[0] = 0.0;
            bctlen = 1;
            bctt[0] = 0.0;
            bcttlen = 1;
        }

        if (adxtail != 0.0) {
            int temp16alen = ScaleExpansionZeroElim(bctlen, bct, adxtail, temp16a);
            int temp32alen = ScaleExpansionZeroElim(temp16alen, temp16a, 2.0 * adx, temp32a);
            int temp8len = ScaleExpansionZeroElim(bcttlen, bctt, adxtail, temp8);
            int temp16blen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * adx, temp16b);
            int temp48len = FastExpansionSumZeroElim(temp32alen, temp32a, temp16blen, temp16b, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            swap_fin();

            if (bdytail != 0.0) {
                int temp8len2 = ScaleExpansionZeroElim(4, cc, adxtail, temp8);
                int temp16clen = ScaleExpansionZeroElim(temp8len2, temp8, bdytail, temp16c);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp16clen, temp16c, finother);
                swap_fin();
            }
            if (cdytail != 0.0) {
                int temp8len2 = ScaleExpansionZeroElim(4, bb, -adxtail, temp8);
                int temp16clen = ScaleExpansionZeroElim(temp8len2, temp8, cdytail, temp16c);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp16clen, temp16c, finother);
                swap_fin();
            }

            int temp32blen = ScaleExpansionZeroElim(temp16alen, temp16a, adxtail, temp32b);
            int temp16clen2 = ScaleExpansionZeroElim(temp8len, temp8, adxtail, temp16c);
            int temp64len = FastExpansionSumZeroElim(temp32blen, temp32b, temp16clen2, temp16c, temp64);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
            swap_fin();
        }
        if (adytail != 0.0) {
            int temp16alen = ScaleExpansionZeroElim(bctlen, bct, adytail, temp16a);
            int temp32alen = ScaleExpansionZeroElim(temp16alen, temp16a, 2.0 * ady, temp32a);
            int temp8len = ScaleExpansionZeroElim(bcttlen, bctt, adytail, temp8);
            int temp16blen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * ady, temp16b);
            int temp48len = FastExpansionSumZeroElim(temp32alen, temp32a, temp16blen, temp16b, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            swap_fin();

            int temp32blen = ScaleExpansionZeroElim(temp16alen, temp16a, adytail, temp32b);
            int temp16clen2 = ScaleExpansionZeroElim(temp8len, temp8, adytail, temp16c);
            int temp64len = FastExpansionSumZeroElim(temp32blen, temp32b, temp16clen2, temp16c, temp64);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
            swap_fin();
        }
    }

    if ((bdxtail != 0.0) || (bdytail != 0.0)) {
        int catlen = 0, cattlen = 0;
        double cat[8], catt[4];
        if ((cdxtail != 0.0) || (cdytail != 0.0) || (adxtail != 0.0) || (adytail != 0.0)) {
            TwoProduct(cdxtail, ady, s1, s0);
            TwoProduct(cdx, adytail, t1, t0);
            TwoTwoSum(s1, s0, t1, t0, u[3], u[2], u[1], u[0]);
            TwoProduct(adxtail, -cdy, s1, s0);
            TwoProduct(adx, -cdytail, t1, t0);
            TwoTwoSum(s1, s0, t1, t0, temp8[3], temp8[2], temp8[1], temp8[0]);
            catlen = FastExpansionSumZeroElim(4, u, 4, temp8, cat);

            TwoProduct(cdxtail, adytail, s1, s0);
            TwoProduct(adxtail, cdytail, t1, t0);
            TwoTwoDiff(s1, s0, t1, t0, catt[3], catt[2], catt[1], catt[0]);
            cattlen = 4;
        } else {
            cat[0] = 0.0;
            catlen = 1;
            catt[0] = 0.0;
            cattlen = 1;
        }

        if (bdxtail != 0.0) {
            int temp16alen = ScaleExpansionZeroElim(catlen, cat, bdxtail, temp16a);
            int temp32alen = ScaleExpansionZeroElim(temp16alen, temp16a, 2.0 * bdx, temp32a);
            int temp8len = ScaleExpansionZeroElim(cattlen, catt, bdxtail, temp8);
            int temp16blen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * bdx, temp16b);
            int temp48len = FastExpansionSumZeroElim(temp32alen, temp32a, temp16blen, temp16b, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            swap_fin();

            if (cdytail != 0.0) {
                int temp8len2 = ScaleExpansionZeroElim(4, aa, bdxtail, temp8);
                int temp16clen = ScaleExpansionZeroElim(temp8len2, temp8, cdytail, temp16c);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp16clen, temp16c, finother);
                swap_fin();
            }
            if (adytail != 0.0) {
                int temp8len2 = ScaleExpansionZeroElim(4, cc, -bdxtail, temp8);
                int temp16clen = ScaleExpansionZeroElim(temp8len2, temp8, adytail, temp16c);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp16clen, temp16c, finother);
                swap_fin();
            }

            int temp32blen = ScaleExpansionZeroElim(temp16alen, temp16a, bdxtail, temp32b);
            int temp16clen2 = ScaleExpansionZeroElim(temp8len, temp8, bdxtail, temp16c);
            int temp64len = FastExpansionSumZeroElim(temp32blen, temp32b, temp16clen2, temp16c, temp64);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
            swap_fin();
        }
        if (bdytail != 0.0) {
            int temp16alen = ScaleExpansionZeroElim(catlen, cat, bdytail, temp16a);
            int temp32alen = ScaleExpansionZeroElim(temp16alen, temp16a, 2.0 * bdy, temp32a);
            int temp8len = ScaleExpansionZeroElim(cattlen, catt, bdytail, temp8);
            int temp16blen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * bdy, temp16b);
            int temp48len = FastExpansionSumZeroElim(temp32alen, temp32a, temp16blen, temp16b, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            swap_fin();

            int temp32blen = ScaleExpansionZeroElim(temp16alen, temp16a, bdytail, temp32b);
            int temp16clen2 = ScaleExpansionZeroElim(temp8len, temp8, bdytail, temp16c);
            int temp64len = FastExpansionSumZeroElim(temp32blen, temp32b, temp16clen2, temp16c, temp64);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
            swap_fin();
        }
    }

    if ((cdxtail != 0.0) || (cdytail != 0.0)) {
        int abtlen = 0, abttlen = 0;
        double abt[8], abtt[4];
        if ((adxtail != 0.0) || (adytail != 0.0) || (bdxtail != 0.0) || (bdytail != 0.0)) {
            TwoProduct(adxtail, bdy, s1, s0);
            TwoProduct(adx, bdytail, t1, t0);
            TwoTwoSum(s1, s0, t1, t0, u[3], u[2], u[1], u[0]);
            TwoProduct(bdxtail, -ady, s1, s0);
            TwoProduct(bdx, -adytail, t1, t0);
            TwoTwoSum(s1, s0, t1, t0, temp8[3], temp8[2], temp8[1], temp8[0]);
            abtlen = FastExpansionSumZeroElim(4, u, 4, temp8, abt);

            TwoProduct(adxtail, bdytail, s1, s0);
            TwoProduct(bdxtail, adytail, t1, t0);
            TwoTwoDiff(s1, s0, t1, t0, abtt[3], abtt[2], abtt[1], abtt[0]);
            abttlen = 4;
        } else {
            abt[0] = 0.0;
            abtlen = 1;
            abtt[0] = 0.0;
            abttlen = 1;
        }

        if (cdxtail != 0.0) {
            int temp16alen = ScaleExpansionZeroElim(abtlen, abt, cdxtail, temp16a);
            int temp32alen = ScaleExpansionZeroElim(temp16alen, temp16a, 2.0 * cdx, temp32a);
            int temp8len = ScaleExpansionZeroElim(abttlen, abtt, cdxtail, temp8);
            int temp16blen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * cdx, temp16b);
            int temp48len = FastExpansionSumZeroElim(temp32alen, temp32a, temp16blen, temp16b, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            swap_fin();

            if (adytail != 0.0) {
                int temp8len2 = ScaleExpansionZeroElim(4, bb, cdxtail, temp8);
                int temp16clen = ScaleExpansionZeroElim(temp8len2, temp8, adytail, temp16c);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp16clen, temp16c, finother);
                swap_fin();
            }
            if (bdytail != 0.0) {
                int temp8len2 = ScaleExpansionZeroElim(4, aa, -cdxtail, temp8);
                int temp16clen = ScaleExpansionZeroElim(temp8len2, temp8, bdytail, temp16c);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp16clen, temp16c, finother);
                swap_fin();
            }

            int temp32blen = ScaleExpansionZeroElim(temp16alen, temp16a, cdxtail, temp32b);
            int temp16clen2 = ScaleExpansionZeroElim(temp8len, temp8, cdxtail, temp16c);
            int temp64len = FastExpansionSumZeroElim(temp32blen, temp32b, temp16clen2, temp16c, temp64);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
            swap_fin();
        }
        if (cdytail != 0.0) {
            int temp16alen = ScaleExpansionZeroElim(abtlen, abt, cdytail, temp16a);
            int temp32alen = ScaleExpansionZeroElim(temp16alen, temp16a, 2.0 * cdy, temp32a);
            int temp8len = ScaleExpansionZeroElim(abttlen, abtt, cdytail, temp8);
            int temp16blen = ScaleExpansionZeroElim(temp8len, temp8, 2.0 * cdy, temp16b);
            int temp48len = FastExpansionSumZeroElim(temp32alen, temp32a, temp16blen, temp16b, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            swap_fin();

            int temp32blen = ScaleExpansionZeroElim(temp16alen, temp16a, cdytail, temp32b);
            int temp16clen2 = ScaleExpansionZeroElim(temp8len, temp8, cdytail, temp16c);
            int temp64len = FastExpansionSumZeroElim(temp32blen, temp32b, temp16clen2, temp16c, temp64);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
            swap_fin();
        }
    }

    (void)aalen;
    (void)bblen;
    (void)cclen;
    return finnow[finlength - 1];
}

}  // namespace

void ExactPredicates::Initialize() noexcept {
    std::call_once(g_init_once_flag, []() noexcept {
        volatile double half = 0.5;
        volatile double epsilon = 1.0;
        volatile double splitter = 1.0;
        volatile double check = 1.0;
        volatile double lastcheck = 1.0;
        bool every_other = true;

        do {
            lastcheck = check;
            epsilon *= half;
            if (every_other) {
                splitter *= 2.0;
            }
            every_other = !every_other;
            check = 1.0 + epsilon;
        } while ((check != 1.0) && (check != lastcheck));

        splitter += 1.0;
        g_epsilon = epsilon;
        g_splitter = splitter;

        g_resulterrbound = (3.0 + 8.0 * g_epsilon) * g_epsilon;
        g_ccwerrboundA = (3.0 + 16.0 * g_epsilon) * g_epsilon;
        g_ccwerrboundB = (2.0 + 12.0 * g_epsilon) * g_epsilon;
        g_ccwerrboundC = (9.0 + 64.0 * g_epsilon) * g_epsilon * g_epsilon;
        g_iccerrboundA = (10.0 + 96.0 * g_epsilon) * g_epsilon;
        g_iccerrboundB = (4.0 + 48.0 * g_epsilon) * g_epsilon;
        g_iccerrboundC = (44.0 + 576.0 * g_epsilon) * g_epsilon * g_epsilon;
    });
}

double ExactPredicates::Orient2D(
    double ax, double ay,
    double bx, double by,
    double cx, double cy) noexcept {
    if (g_splitter == 0.0) {
        Initialize();
    }

    const double detleft = (ax - cx) * (by - cy);
    const double detright = (ay - cy) * (bx - cx);
    const double det = detleft - detright;

    double detsum = 0.0;
    if (detleft > 0.0) {
        if (detright <= 0.0) {
            return det;
        }
        detsum = detleft + detright;
    } else if (detleft < 0.0) {
        if (detright >= 0.0) {
            return det;
        }
        detsum = -detleft - detright;
    } else {
        return det;
    }

    const double errbound = g_ccwerrboundA * detsum;
    if ((det >= errbound) || (-det >= errbound)) {
        return det;
    }

    return Orient2DAdapt(ax, ay, bx, by, cx, cy, detsum);
}

double ExactPredicates::InCircle(
    double ax, double ay,
    double bx, double by,
    double cx, double cy,
    double dx, double dy) noexcept {
    if (g_splitter == 0.0) {
        Initialize();
    }

    const double adx = ax - dx;
    const double bdx = bx - dx;
    const double cdx = cx - dx;
    const double ady = ay - dy;
    const double bdy = by - dy;
    const double cdy = cy - dy;

    const double bdxcdy = bdx * cdy;
    const double cdxbdy = cdx * bdy;
    const double alift = adx * adx + ady * ady;

    const double cdxady = cdx * ady;
    const double adxcdy = adx * cdy;
    const double blift = bdx * bdx + bdy * bdy;

    const double adxbdy = adx * bdy;
    const double bdxady = bdx * ady;
    const double clift = cdx * cdx + cdy * cdy;

    const double det = alift * (bdxcdy - cdxbdy) +
                       blift * (cdxady - adxcdy) +
                       clift * (adxbdy - bdxady);

    const double permanent = (Absolute(bdxcdy) + Absolute(cdxbdy)) * alift +
                             (Absolute(cdxady) + Absolute(adxcdy)) * blift +
                             (Absolute(adxbdy) + Absolute(bdxady)) * clift;
    const double errbound = g_iccerrboundA * permanent;
    if ((det > errbound) || (-det > errbound)) {
        return det;
    }

    return InCircleAdapt(ax, ay, bx, by, cx, cy, dx, dy, permanent);
}

int ExactPredicates::PointInTriangle2D(
    const Vec3d& a, const Vec3d& b, const Vec3d& c, const Vec3d& p) noexcept {
    double o1 = Orient2D(a, b, p);
    double o2 = Orient2D(b, c, p);
    double o3 = Orient2D(c, a, p);

    if (Orient2D(a, b, c) < 0.0) {
        o1 = -o1;
        o2 = -o2;
        o3 = -o3;
    }

    if (o1 < 0.0 || o2 < 0.0 || o3 < 0.0) {
        return 0;
    }
    if (o1 == 0.0 || o2 == 0.0 || o3 == 0.0) {
        return 2;
    }
    return 1;
}

bool ExactPredicates::SegmentsProperlyIntersect2D(
    const Vec3d& a, const Vec3d& b,
    const Vec3d& c, const Vec3d& d) noexcept {
    const double o1 = Orient2D(a, b, c);
    const double o2 = Orient2D(a, b, d);
    const double o3 = Orient2D(c, d, a);
    const double o4 = Orient2D(c, d, b);

    return ((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) &&
           ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0));
}

double ExactPredicates::InterpolateZOnTriangle(
    const Vec3d& a, const Vec3d& b, const Vec3d& c,
    double px, double py) noexcept {
    const double denom = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    if (Absolute(denom) <= 1e-15) {
        return (a.z + b.z + c.z) / 3.0;
    }
    const double w1 = ((b.y - c.y) * (px - c.x) + (c.x - b.x) * (py - c.y)) / denom;
    const double w2 = ((c.y - a.y) * (px - c.x) + (a.x - c.x) * (py - c.y)) / denom;
    const double w3 = 1.0 - w1 - w2;
    return w1 * a.z + w2 * b.z + w3 * c.z;
}

}  // namespace geo_kernel

