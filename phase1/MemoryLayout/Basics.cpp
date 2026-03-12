
/*


- sizeof(), alignof(), offsetof()

*/

#include<iostream>

struct Alignment {
    // double d;
    // char c;
    // int i;
    double d;
    int b;
    char a;     // 1byte
    char c;     // 1 byte
};

int main() {

    Alignment aln;

    std::cout<<"size: "<<sizeof(aln)<<std::endl;
    std::cout<<"aligh of a,b,c "<<alignof(aln.a)<<" "<<alignof(aln.b)<<" "<<alignof(aln.c)<<" "<<alignof(aln.d)<<std::endl;
    std::cout<<"offset of a,b,c "<<offsetof(Alignment, d)<<" "<<offsetof(Alignment, c)<<" "<<offsetof(Alignment, a)<<std::endl;

    alignas(8) Alignment aln2;
    std::cout<<"alignment: "<<sizeof(aln2)<<std::endl;
    return 0;
}

