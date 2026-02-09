#include <cstdint>
#include <climits>
#include <array>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <sstream>
#define bitLimit 65536

/*
all imms are 16 bit
all reg are 16 bit or 8 bit
registor id's are 8 bit

register names:
A, B, C, SP,  // 16 bit
G, H, I, FLAGS,  // 8 bit

0, 1, 2, 3,
4, 5, 6, 7, 

FFFE$FFFF = reset vector
FEFE-FFFD = stack; start from FFFD grow towards 0000

system calls [TOGGLEABLE]

FEFD  : invoke call [1 : PRINTCHAR          ]
FEFC  : arg1        [CHAR(only FEFB is used)]

000000  :nop
000010  :load @imm, reg  // value at imm = reg
000011  :load reg, @imm // reg = value at imm
000100  :load @imm+reg, reg // value at (imm+reg) = reg
000101  :load reg, @imm+reg // reg = value at (imm+reg)
000110  :load reg, imm   // reg = imm
000111  :load @imm, imm // value at imm = imm

001000  :jmp imm // pc = imm
001001  :jmp reg+imm // pc = (reg + imm)

001010  :cmp reg, reg // compare 2 registors
001011  :cmp @imm, @imm // compare 2 values in memory
001100  :cmp reg, @imm // compare a reg with a value in memory
001101  :cmp reg, imm // compare a reg with a imm
// fyi compare is just subtract with extra steps

001110  :eq Vimm // treat imm as signed, pc += imm if zero flag on       // ==
001111  :nq Vimm // treat imm as signed, pc += imm if zero flag off       // !=
010000  :ls Vimm // treat imm as signed, pc += imm if carry flag off       // <
010001  :mreq Vimm // treat imm as signed, pc += imm if carry flag on // >=
010010  :ov Vimm // treat imm as signed, pc += imm if overflow flag on 
010011  :nov Vimm // treat imm as signed, pc += imm if overflow lag off

010100  :add reg, reg, reg1 // reg1 = reg + reg 
010101  :add reg, @imm, reg1 // reg1 = reg + (value at imm)
010110  :addi reg, imm // reg += imm
// also sets flags accordingly

010111  :sub reg, reg, reg1 // reg1 = reg - reg 
011000  :sub reg, @imm, reg1 // reg1 = reg - (value at imm)
011001  :subi reg, imm // reg -= imm
// also sets flags accordingly

011010  :mul reg, reg, reg1 // reg1 = reg + reg 
011011  :mul reg, @imm, reg1 // reg1 = reg + (value at imm)
// does not set flags

011100  :div reg, reg, reg1 // reg1 = reg + reg 
011101  :div reg, @imm, reg1 // reg1 = reg + (value at imm)
// does not set flags

011110  :call reg+imm
011111  :ret
100000  :push imm    // push a imm onto stack
100001  :push @imm // push the value at a imm to stack
100010  :push reg     // push a reg onto stack
100011  :pop @imm // pop from stack to some point in memory ie memory[imm] = pop()
100100  :pop reg // reg = pop()


assmbler extras:
.bytes int, int, int ...; name

example
    .bytes 104, 105, 33, 10; message


.string "Hello, world"; name

parent_lable:
    .child_lable
        .child_child_lable

example
    main:
        .print
            code...
        .input
            code...
        .loop
            call print
            jmp loop

--- assembler examples ---
hello, world!

.string "Hello, world\n" message
main:
    load A, message      // a is equal to pointer to first letter
    .loop
        load G, @0+A     // load g with the char
        cmp G, 0         // if compare g and 0
        eq exit          // if g == 0, jump to exit
        load @65276, G   // load char into 0xFEFC
        load @65277, 1   // load 1 into 0xFEFD; invokes syscall, prints
        addi A, 1        // a++
        jmp .loop        // goto loop
exit:                
    jmp exit             // loop forever
.reset main



*/

int safeFloatTo16ui(float f) {
    if (f > INT16_MAX) return INT16_MAX;
    if (f < INT16_MIN) return INT16_MIN;
    return (uint16_t)f;
}

enum Flags : uint8_t {
    FLAG_C = 1 << 0, // Carry / no borrow
    FLAG_Z = 1 << 1, // Zero
    FLAG_N = 1 << 2, // Negative
    FLAG_V = 1 << 3  // Overflow
};

enum Opcode : char {
    nop                 = 0b000000,

    loadAtImmReg        = 0b000010,
    loadRegAtImm        = 0b000011,
    loadAtImmRegReg     = 0b000100,
    loadRegAtImmReg     = 0b000101,
    loadRegImm          = 0b000110,
    loadAtImmImm        = 0b000111,

    jmpImm            = 0b001000,
    jmpRegImm           = 0b001001,

    cmpRegReg           = 0b001010,
    cmpAtImmAtImm       = 0b001011,
    cmpRegAtImm         = 0b001100,
    cmpRegImm           = 0b001101,

    eqVimm              = 0b001110,
    nqVimm              = 0b001111,
    lsVimm              = 0b010000,
    mreqVimm            = 0b010001,
    ovVimm              = 0b010010,
    novVimm             = 0b010011,

    addRegRegReg        = 0b010100,
    addRegAtImmReg      = 0b010101,
    addiRegImm          = 0b010110,

    subRegRegReg        = 0b010111,
    subRegAtImmReg      = 0b011000,
    subiRegImm          = 0b011001,

    mulRegRegReg        = 0b011010,
    mulRegAtImmReg      = 0b011011,

    divRegRegReg        = 0b011100,
    divRegAtImmReg      = 0b011101,

    callRegImm          = 0b011110,
    ret                 = 0b011111,

    pushImm             = 0b100000,
    pushAtImm           = 0b100001,
    pushReg             = 0b100010,

    popAtImm            = 0b100011,
    popReg              = 0b100100
};

struct DASH {
    Opcode opcode;
    uint16_t pc;

    const int stackBegin = 0xFFFD;
    const int SP = 3;

    struct Registers {
        uint16_t r16[4]; // A B C SP
        uint8_t  r8[4];  // G H I FLAGS

        inline uint16_t read(int idx) const {
            int i = idx & 0x3;
            return (idx & 0x4) ? r8[i] : r16[i];
        }

        inline void write(int idx, uint16_t v) {
            int i = idx & 0x3;
            if (idx & 0x4)
                r8[i] = static_cast<uint8_t>(v);
            else
                r16[i] = v;
        }

        inline uint16_t read16(int idx) const {
            return r16[idx & 0x3];
        }

        inline void write16(int idx, uint16_t v) {
            r16[idx & 0x3] = v;
        }

        inline uint8_t read8(int idx) const {
            return r8[idx & 0x3];
        }

        inline void write8(int idx, uint8_t v) {
            r8[idx & 0x3] = v;
        }

        inline void setFlagsOnSub(uint16_t lhs, uint16_t rhs) {
            uint8_t flags = 0;
            uint16_t result = lhs - rhs;

            // Carry = NO borrow (unsigned compare)
            if (lhs >= rhs)
                flags |= FLAG_C;

            // Zero
            if ((result & 0xFFFF) == 0)
                flags |= FLAG_Z;

            // Negative (sign bit)
            if (result & 0x8000)
                flags |= FLAG_N;

            // Overflow (signed)
            if (((lhs ^ rhs) & (lhs ^ result) & 0x8000) != 0)
                flags |= FLAG_V;

            r8[3] = flags;
        }

        inline void setFlagsOnAdd(uint16_t lhs, uint16_t rhs) {
            uint16_t result = static_cast<uint16_t>(lhs + rhs);
            uint8_t flags = 0;

            // Carry
            if (lhs + rhs > 0xFFFF)
                flags |= FLAG_C;

            // Zero
            if (result == 0)
                flags |= FLAG_Z;

            // Negative
            if (result & 0x8000)
                flags |= FLAG_N;

            // Overflow (signed)
            if (((~(lhs ^ rhs)) & (lhs ^ result) & 0x8000) != 0)
                flags |= FLAG_V;

            r8[3] = flags;
        }

        inline Flags readFlag(Flags flag) const {
            return (r8[3] & flag) ? flag : static_cast<Flags>(0);
        }
    };

    Registers regs;
    std::array<uint8_t, bitLimit> memory;

    inline uint16_t byteWord(uint8_t hi, uint8_t lo) {
        return (uint16_t(hi) << 8) | lo;
    }

    inline uint16_t grabWord() {
        return byteWord(memory[pc++], memory[pc++]);
    }

    inline uint16_t readWord(uint16_t addr) {
        return byteWord(memory[addr], memory[addr+1]);
    }

    inline void writeWord(uint16_t addr, uint16_t word) {
        memory[addr] = static_cast<uint8_t>(word>>8);
        memory[addr+1] = static_cast<uint8_t>(word & 255);
    }
    
    inline uint16_t spAddr() const {
        return stackBegin - regs.read16(SP);
    }

    void push(uint8_t v) {
        memory[spAddr()] = v;
        regs.write16(SP, regs.read16(SP) + 1);
    }

    uint8_t pop() {
        regs.write16(SP, regs.read16(SP) - 1);
        return memory[spAddr()];
    }

    void run() {
        pc = byteWord(memory[0xFFFF], memory[0xFFFE]);
        while (true) {
            if (memory[0xFEFD]) {
                switch (memory[0xFEFD]) {
                    case 1:
                        std::cout << memory[0xFEFC];
                }
                memory[0xFEFD] = 0;
            }

            opcode = (Opcode)memory[pc++];

            switch (opcode) {
            case nop:
                break;

            case loadAtImmReg: {
                uint16_t addr = grabWord();
                uint8_t r = memory[pc++];
                uint16_t v = regs.read(r);
                memory[addr] = static_cast<uint8_t>(v);
                if ((r & 0x4) == 0)
                    memory[addr + 1] = static_cast<uint8_t>(v >> 8);
                break;
            }

            case loadRegAtImm: {
                uint16_t addr = grabWord();
                uint8_t r = memory[pc++];
                uint16_t v = memory[addr];
                if ((r & 0x4) == 0)
                    v |= static_cast<uint16_t>(memory[addr + 1]) << 8;
                regs.write(r, v);
                break;
            }

            case loadAtImmRegReg: {
                uint16_t base = grabWord();
                uint16_t addr = base + regs.read(memory[pc++]);
                uint8_t r = memory[pc++];
                uint16_t v = regs.read(r);
                memory[addr] = static_cast<uint8_t>(v);
                if ((r & 0x4) == 0)
                    memory[addr + 1] = static_cast<uint8_t>(v >> 8);
                break;
            }

            case loadRegAtImmReg: {
                uint16_t base = grabWord();
                uint16_t addr = base + regs.read(memory[pc++]);
                uint8_t r = memory[pc++];
                uint16_t v = memory[addr];
                if ((r & 0x4) == 0)
                    v |= static_cast<uint16_t>(memory[addr + 1]) << 8;
                regs.write(r, v);
                break;
            }

            case loadRegImm: {
                uint8_t r = memory[pc++];
                regs.write(r, grabWord());
                break;
            }

            case loadAtImmImm: {
                uint16_t addr = grabWord();
                uint16_t v = grabWord();
                memory[addr] = static_cast<uint8_t>(v);
                memory[addr + 1] = static_cast<uint8_t>(v >> 8);
                break;
            }

            case jmpImm:
                pc = grabWord();
                break;

            case jmpRegImm: {
                uint16_t base = grabWord();
                uint16_t addr = base + regs.read(memory[pc++]);
                pc = readWord(addr);
                break;
            }

            case cmpRegReg:
                regs.setFlagsOnSub(regs.read(memory[pc++]),
                                regs.read(memory[pc++]));
                break;

            case cmpAtImmAtImm: {
                uint16_t a = grabWord();
                uint16_t b = grabWord();
                regs.setFlagsOnSub(memory[a], memory[b]);
                break;
            }

            case cmpRegAtImm: {
                uint16_t v = regs.read(memory[pc++]);
                regs.setFlagsOnSub(v, memory[grabWord()]);
                break;
            }

            case cmpRegImm: {
                uint16_t v = regs.read(memory[pc++]);
                regs.setFlagsOnSub(v, grabWord());
                break;
            }

            case eqVimm: {
                int16_t off = static_cast<int16_t>(grabWord());
                if (regs.readFlag(FLAG_Z)) pc += off;
                break;
            }

            case nqVimm: {
                int16_t off = static_cast<int16_t>(grabWord());
                if (!regs.readFlag(FLAG_Z)) pc += off;
                break;
            }

            case lsVimm: {
                int16_t off = static_cast<int16_t>(grabWord());
                if (!regs.readFlag(FLAG_C)) pc += off;
                break;
            }

            case mreqVimm: {
                int16_t off = static_cast<int16_t>(grabWord());
                if (regs.readFlag(FLAG_C)) pc += off;
                break;
            }

            case ovVimm: {
                int16_t off = static_cast<int16_t>(grabWord());
                if (regs.readFlag(FLAG_V)) pc += off;
                break;
            }

            case novVimm: {
                int16_t off = static_cast<int16_t>(grabWord());
                if (!regs.readFlag(FLAG_V)) pc += off;
                break;
            }

            case addRegRegReg: {
                uint16_t a = regs.read(memory[pc++]);
                uint16_t b = regs.read(memory[pc++]);
                uint8_t d = memory[pc++];
                regs.write(d, a + b);
                regs.setFlagsOnAdd(a, b);
                break;
            }

            case addRegAtImmReg: {
                uint16_t a = regs.read(memory[pc++]);
                uint16_t b = memory[grabWord()];
                uint8_t d = memory[pc++];
                regs.write(d, a + b);
                regs.setFlagsOnAdd(a, b);
                break;
            }

            case addiRegImm: {
                uint8_t r = memory[pc++];
                uint16_t v = regs.read(r);
                uint16_t imm = grabWord();
                regs.write(r, v + imm);
                regs.setFlagsOnAdd(v, imm);
                break;
            }

            case subRegRegReg: {
                uint16_t a = regs.read(memory[pc++]);
                uint16_t b = regs.read(memory[pc++]);
                uint8_t d = memory[pc++];
                regs.write(d, a - b);
                regs.setFlagsOnSub(a, b);
                break;
            }

            case subRegAtImmReg: {
                uint16_t a = regs.read(memory[pc++]);
                uint16_t b = memory[grabWord()];
                uint8_t d = memory[pc++];
                regs.write(d, a - b);
                regs.setFlagsOnSub(a, b);
                break;
            }

            case subiRegImm: {
                uint8_t r = memory[pc++];
                uint16_t v = regs.read(r);
                uint16_t imm = grabWord();
                regs.write(r, v - imm);
                regs.setFlagsOnSub(v, imm);
                break;
            }

            case mulRegRegReg: {
                uint16_t a = regs.read(memory[pc++]);
                uint16_t b = regs.read(memory[pc++]);
                uint8_t d = memory[pc++];
                regs.write(d, static_cast<uint16_t>(a * b));
                break;
            }

            case mulRegAtImmReg: {
                uint16_t a = regs.read(memory[pc++]);
                uint16_t b = memory[grabWord()];
                uint8_t d = memory[pc++];
                regs.write(d, static_cast<uint16_t>(a * b));
                break;
            }

            case divRegRegReg: {
                uint16_t a = regs.read(memory[pc++]);
                uint16_t b = regs.read(memory[pc++]);
                uint8_t d = memory[pc++];
                regs.write(d, b ? static_cast<uint16_t>(a / b) : 0);
                break;
            }

            case divRegAtImmReg: {
                uint16_t a = regs.read(memory[pc++]);
                uint16_t b = memory[grabWord()];
                uint8_t d = memory[pc++];
                regs.write(d, b ? static_cast<uint16_t>(a / b) : 0);
                break;
            }

            case callRegImm: {
                uint16_t base = regs.read(memory[pc++]);
                uint16_t target = base + grabWord();

                uint16_t retpc = pc;
                uint8_t flags = regs.read8(3); // FLAGS

                push(flags);
                push(static_cast<uint8_t>(retpc >> 8));
                push(static_cast<uint8_t>(retpc));

                pc = target;
                break;
            }

            case ret: {
                uint16_t pcLO = pop();
                uint16_t pcHI = pop();
                uint8_t flags = pop();

                pc = byteWord(pcHI, pcLO);
                regs.write8(3, flags); // FLAGS

                break;
            }

            case pushImm: {
                uint16_t v = grabWord();
                push(static_cast<uint8_t>(v));
                push(static_cast<uint8_t>(v >> 8));
                break;
            }

            case pushAtImm: {
                uint16_t v = memory[grabWord()];
                push(static_cast<uint8_t>(v));
                push(static_cast<uint8_t>(v >> 8));
                break;
            }

            case pushReg: {
                uint16_t v = regs.read(memory[pc++]);
                push(static_cast<uint8_t>(v));
                push(static_cast<uint8_t>(v >> 8));
                break;
            }

            case popAtImm: {
                uint16_t addr = grabWord();
                memory[addr + 1] = pop();
                memory[addr] = pop();
                break;
            }

            case popReg: {
                uint8_t hi = pop();
                uint8_t lo = pop();
                regs.write(memory[pc++], byteWord(hi, lo));
                break;
            }
            }
        }
    }
};

struct Assmbler {
    struct Fixup {
        std::string label;
        uint16_t at;
        bool relative;
    };

    static constexpr uint16_t STACK_LOW  = 0xFEFE;
    static constexpr uint16_t STACK_HIGH = 0xFFFD;
    static constexpr uint16_t RESET_VEC  = 0xFFFE;

    static uint8_t regId(const std::string& r) {
        if (r=="A") return 0;
        if (r=="B") return 1;
        if (r=="C") return 2;
        if (r=="SP") return 3;
        if (r=="G") return 4;
        if (r=="H") return 5;
        if (r=="I") return 6;
        if (r=="FLAGS") return 7;
        throw std::runtime_error("bad register");
    }

    static bool isNumber(const std::string& s) {
        if (s.empty()) return false;
        size_t i = (s[0]=='-') ? 1 : 0;
        if (i>=s.size()) return false;
        for (; i<s.size(); i++) if (!isdigit(s[i])) return false;
        return true;
    }

    static uint16_t toU16(const std::string& s) {
        return static_cast<uint16_t>(std::stoi(s));
    }

    uint8_t decodeEscape(char c) {
        switch (c) {
            case 'n': return 0x0A;
            case 'r': return 0x0D;
            case 't': return 0x09;
            case '0': return 0x00;
            case '\\': return 0x5C;
            case '"': return 0x22;
            default: throw std::runtime_error("bad escape");
        }
    }

    static std::vector<std::string> splitLines(const std::string& code) {
        std::vector<std::string> lines;
        std::string current;

        bool inString = false;
        char stringDelim = 0;
        bool escape = false;

        for (size_t i = 0; i < code.size(); ++i) {
            char c = code[i];

            if (escape) {
                current.push_back(c);
                escape = false;
                continue;
            }

            if (c == '\\' && inString) {
                current.push_back(c);
                escape = true;
                continue;
            }

            if ((c == '"' || c == '\'') && !escape) {
                if (!inString) {
                    inString = true;
                    stringDelim = c;
                } else if (c == stringDelim) {
                    inString = false;
                }
                current.push_back(c);
                continue;
            }

            if (c == ';' && !inString) {
                while (i < code.size() && code[i] != '\n') i++;
                if (i < code.size()) {
                    lines.push_back(current);
                    current.clear();
                }
                continue;
            }

            if (c == '\n' && !inString) {
                lines.push_back(current);
                current.clear();
                continue;
            }

            current.push_back(c);
        }

        if (!current.empty())
            lines.push_back(current);

        return lines;
    }

    static std::vector<std::string> splitTokens(const std::string& line) {
        std::vector<std::string> tokens;
        std::string current;

        bool inString = false;
        char stringDelim = 0;
        bool escape = false;

        auto flush = [&]() {
            if (!current.empty()) {
                tokens.push_back(current);
                current.clear();
            }
        };

        for (size_t i = 0; i < line.size(); ++i) {
            char c = line[i];

            if (escape) {
                current.push_back(c);
                escape = false;
                continue;
            }

            if (c == '\\' && inString) {
                current.push_back(c);
                escape = true;
                continue;
            }

            if ((c == '"' || c == '\'') && !escape) {
                if (!inString) {
                    inString = true;
                    stringDelim = c;
                } else if (c == stringDelim) {
                    inString = false;
                }
                current.push_back(c);
                continue;
            }

            if (!inString && (c == ' ' || c == '\t' || c == ',')) {
                flush();
                continue;
            }

            current.push_back(c);
        }

        flush();
        return tokens;
    }

    std::array<uint8_t, bitLimit> assmble(const std::string& code) {
        std::array<uint8_t, bitLimit> out{};
        std::unordered_map<std::string,uint16_t> labels;
        std::vector<Fixup> fixups;

        uint16_t pc = 0;
        std::string scope;
        std::string pendingReset;

        auto ensureNotStack = [&](uint16_t addr){
            if (addr>=STACK_LOW && addr<=STACK_HIGH)
                throw std::runtime_error("code or label placed in stack region");
        };

        auto emit8 = [&](uint8_t v){
            std::cout << static_cast<int>(v) << " '" << static_cast<char>(v) << "' @ " << pc << std::endl;
            ensureNotStack(pc);
            out[pc++] = v;
        };

        auto emit16 = [&](uint16_t v){
            emit8(static_cast<uint8_t>(v&255));
            emit8(static_cast<uint8_t>(v>>8));
        };

        auto qualify = [&](const std::string& l){
            if (!l.empty() && l[0]=='.') return scope + l;
            return l;
        };

        auto emitImmOrFixup = [&](const std::string& t, bool rel){
            if (isNumber(t)) emit16(toU16(t));
            else {
                fixups.push_back({qualify(t), pc, rel});
                emit16(0);
            }
        };

        std::vector<std::string> lines = splitLines(code);

        for (const std::string& rawLine : lines) {
            std::vector<std::string> tok = splitTokens(rawLine);
            if (tok.empty()) continue;

            const std::string& op = tok[0];

            if (op == ".org") {
                pc = toU16(tok.at(1));
                ensureNotStack(pc);
                continue;
            }

            if (op == ".reset") {
                pendingReset = tok.at(1);
                continue;
            }

            if (op[0]=='.' && tok.size()==1) {
                labels[qualify(op)] = pc;
                continue;
            }

            if (op.back()==':') {
                std::string lbl = op.substr(0, op.size()-1);
                if (!lbl.empty() && lbl[0] != '.') scope = lbl;
                labels[qualify(lbl)] = pc;
                continue;
            }

            if (op == ".bytes") {
                uint16_t start = pc;
                for (size_t i=1;i<tok.size();i++)
                    emit8(static_cast<uint8_t>(std::stoi(tok[i])));
                if (!tok.empty())
                    labels[tok.back()] = start;
                continue;
            }

            if (op == ".string") {
                const std::string& s = tok.at(1);
                uint16_t start = pc;

                for (size_t i=1;i+1<s.size();i++) {
                    if (s[i]=='\\') {
                        i++;
                        emit8(decodeEscape(s[i]));
                    } else {
                        emit8(static_cast<uint8_t>(s[i]));
                    }
                }
                emit8(0);

                if (tok.size() >= 3)
                    labels[tok[2]] = start;
                continue;
            }

            if (op == "nop") { emit8(nop); continue; }

            if (op == "load") {
                const std::string& a = tok.at(1);
                const std::string& b = tok.at(2);

                if (a[0]=='@' && a.find('+')!=std::string::npos) {
                    auto p=a.find('+');
                    emit8(loadAtImmRegReg);
                    emit16(toU16(a.substr(1,p-1)));
                    emit8(regId(a.substr(p+1)));
                    emit8(regId(b));
                }
                else if (b[0]=='@' && b.find('+')!=std::string::npos) {
                    auto p=b.find('+');
                    emit8(loadRegAtImmReg);
                    emit16(toU16(b.substr(1,p-1)));
                    emit8(regId(b.substr(p+1)));
                    emit8(regId(a));
                }
                else if (a[0]=='@' && isNumber(b)) {
                    emit8(loadAtImmImm);
                    emit16(toU16(a.substr(1)));
                    emit16(toU16(b));
                }
                else if (a[0]=='@') {
                    emit8(loadAtImmReg);
                    emit16(toU16(a.substr(1)));
                    emit8(regId(b));
                }
                else if (b[0]=='@') {
                    emit8(loadRegAtImm);
                    emit16(toU16(b.substr(1)));
                    emit8(regId(a));
                }
                else {
                    emit8(loadRegImm);
                    emit8(regId(a));
                    emitImmOrFixup(b,false);
                }
                continue;
            }

            auto emitALU = [&](Opcode rr, Opcode ri){
                const std::string& a = tok.at(1);
                const std::string& b = tok.at(2);
                const std::string& c = tok.at(3);

                if (b[0]=='@') {
                    emit8(ri);
                    emit8(regId(a));
                    emit16(toU16(b.substr(1)));
                    emit8(regId(c));
                } else {
                    emit8(rr);
                    emit8(regId(a));
                    emit8(regId(b));
                    emit8(regId(c));
                }
            };

            if (op=="add") { emitALU(addRegRegReg, addRegAtImmReg); continue; }
            if (op=="sub") { emitALU(subRegRegReg, subRegAtImmReg); continue; }
            if (op=="mul") { emitALU(mulRegRegReg, mulRegAtImmReg); continue; }
            if (op=="div") { emitALU(divRegRegReg, divRegAtImmReg); continue; }

            if (op == "addi") {
                emit8(addiRegImm);
                emit8(regId(tok.at(1)));
                emit16(toU16(tok.at(2)));
                continue;
            }

            if (op == "subi") {
                emit8(subiRegImm);
                emit8(regId(tok.at(1)));
                emit16(toU16(tok.at(2)));
                continue;
            }

            if (op=="jmp") {
                const std::string& t = tok.at(1);
                if (t.find('+')!=std::string::npos) {
                    auto p=t.find('+');
                    emit8(jmpRegImm);
                    emit16(toU16(t.substr(p+1)));
                    emit8(regId(t.substr(0,p)));
                } else {
                    emit8(jmpImm);
                    emitImmOrFixup(t,false);
                }
                continue;
            }

            if (op=="call") {
                const std::string& t = tok.at(1);
                auto p=t.find('+');
                emit8(callRegImm);
                emit8(regId(t.substr(0,p)));
                emit16(toU16(t.substr(p+1)));
                continue;
            }

            if (op=="ret") { emit8(ret); continue; }

            if (op=="cmp") {
                const std::string& a = tok.at(1);
                const std::string& b = tok.at(2);

                if (b[0]=='@') {
                    emit8(cmpRegAtImm);
                    emit8(regId(a));
                    emit16(toU16(b.substr(1)));
                } else if (isNumber(b)) {
                    emit8(cmpRegImm);
                    emit8(regId(a));
                    emit16(toU16(b));
                } else {
                    emit8(cmpRegReg);
                    emit8(regId(a));
                    emit8(regId(b));
                }
                continue;
            }

            auto emitCond = [&](Opcode o){
                emit8(o);
                emitImmOrFixup(tok.at(1), true);
            };

            if (op=="eq")   emitCond(eqVimm);
            if (op=="nq")   emitCond(nqVimm);
            if (op=="ls")   emitCond(lsVimm);
            if (op=="mreq") emitCond(mreqVimm);
            if (op=="ov")   emitCond(ovVimm);
            if (op=="nov")  emitCond(novVimm);

            if (op=="push") {
                const std::string& v = tok.at(1);
                if (v[0]=='@') { emit8(pushAtImm); emit16(toU16(v.substr(1))); }
                else if (isNumber(v)) { emit8(pushImm); emit16(toU16(v)); }
                else { emit8(pushReg); emit8(regId(v)); }
                continue;
            }

            if (op=="pop") {
                const std::string& v = tok.at(1);
                if (v[0]=='@') { emit8(popAtImm); emit16(toU16(v.substr(1))); }
                else { emit8(popReg); emit8(regId(v)); }
                continue;
            }
        }

        for (auto& f: fixups) {
            if (!labels.count(f.label))
                throw std::runtime_error("undefined label: "+f.label);
            uint16_t target = labels[f.label];
            if (f.relative) {
                int16_t d = static_cast<int16_t>(target - (f.at+2));
                out[f.at+1]   = static_cast<uint8_t>(d>>8);
                out[f.at] = static_cast<uint8_t>(d&255);
            } else {
                out[f.at+1]   = static_cast<uint8_t>(target>>8);
                out[f.at] = static_cast<uint8_t>(target&255);
            }
        }

        if (!pendingReset.empty()) {
            if (!labels.count(pendingReset))
                throw std::runtime_error("undefined reset label");
            uint16_t addr = labels[pendingReset];
            out[RESET_VEC+1]   = static_cast<uint8_t>(addr>>8);
            out[RESET_VEC] = static_cast<uint8_t>(addr&255);
        }

        return out;
    }
};

int main() {
    DASH vm;
    Assmbler dashASM;
    std::string code = ".string \"Hello, world\\n\" message\n"
                       "main:\n"
                       "    load A, message\n"      // a is equal to pointer to first letter
                       "    .loop\n"
                       "        load G, @0+A\n"     // load g with the char
                       "        cmp G, 0\n"         // if compare g and 0
                       "        eq exit\n"          // if g == 0, jump to exit
                       "        load @65276, G\n"   // load char into 0xFEFC
                       "        load @65277, 1\n"   // load 1 into 0xFEFD; invokes syscall, prints
                       "        addi A, 1\n"        // a++
                       "        jmp .loop\n"        // goto loop
                       "exit:\n"                    
                       "    jmp exit\n"             // loop forever
                       ".reset main\n"
                       ;

    vm.memory = dashASM.assmble(code);
    vm.run();
    return 0;
}