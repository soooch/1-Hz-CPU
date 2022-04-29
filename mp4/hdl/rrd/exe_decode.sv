import rv32i_types::*;
import ctrl_sigs::*;

module alu_decode (
    input uopc::micro_opcode_t uopcode,
    input logic [4:0] shamt,
    output alu_ctrl_sigs_t ctrl
);

    always_comb begin
        unique casez ({uopcode, shamt})
            {uopc::lui, 5'b?????}    : ctrl = '{alufnt::add,  opr2t::imm};

            {uopc::addi,  5'b?????} : ctrl = '{alufnt::add,  opr2t::imm};
            {uopc::slti,  5'b?????} : ctrl = '{alufnt::slt,  opr2t::imm};
            {uopc::sltiu, 5'b?????} : ctrl = '{alufnt::sltu, opr2t::imm};
            {uopc::xori,  5'b?????} : ctrl = '{alufnt::xoro, opr2t::imm};
            {uopc::ori,   5'b?????} : ctrl = '{alufnt::oro,  opr2t::imm};
            {uopc::andi,  5'b?????} : ctrl = '{alufnt::ando, opr2t::imm};
            {uopc::slli,  5'b?????} : ctrl = '{alufnt::sl,   opr2t::imm};
            {uopc::srli,  5'b?????} : ctrl = '{alufnt::sr,   opr2t::imm};
            {uopc::srai,  5'b?????} : ctrl = '{alufnt::sra,  opr2t::imm};

            {uopc::add,   5'b?????} : ctrl = '{alufnt::add,  opr2t::rs2};
            {uopc::sub,   5'b?????} : ctrl = '{alufnt::sub,  opr2t::rs2};
            {uopc::sll,   5'b?????} : ctrl = '{alufnt::sl,   opr2t::rs2};
            {uopc::slt,   5'b?????} : ctrl = '{alufnt::slt,  opr2t::rs2};
            {uopc::sltu,  5'b?????} : ctrl = '{alufnt::sltu, opr2t::rs2};
            {uopc::xoro,  5'b?????} : ctrl = '{alufnt::xoro, opr2t::rs2};
            {uopc::srl,   5'b?????} : ctrl = '{alufnt::sr,   opr2t::rs2};
            {uopc::sra,   5'b?????} : ctrl = '{alufnt::sra,  opr2t::rs2};
            {uopc::oro,   5'b?????} : ctrl = '{alufnt::oro,  opr2t::rs2};
            {uopc::ando,  5'b?????} : ctrl = '{alufnt::ando, opr2t::rs2};

            {uopc::grevi, 5'b?????} : ctrl = '{alufnt::brev, opr2t::imm};

            {uopc::andn,  5'b?????} : ctrl = '{alufnt::andn, opr2t::rs2};
            {uopc::orn,   5'b?????} : ctrl = '{alufnt::orn,  opr2t::rs2};
            {uopc::xnoro, 5'b?????} : ctrl = '{alufnt::xorn, opr2t::rs2};

            {uopc::min,   5'b?????} : ctrl = '{alufnt::min,  opr2t::rs2};
            {uopc::minu,  5'b?????} : ctrl = '{alufnt::minu, opr2t::rs2};
            {uopc::max,   5'b?????} : ctrl = '{alufnt::max,  opr2t::rs2};
            {uopc::maxu,  5'b?????} : ctrl = '{alufnt::maxu, opr2t::rs2};

            {uopc::cbsxt, 5'b??000} : ctrl = '{alufnt::clz,  opr2t::imm};
            {uopc::cbsxt, 5'b??001} : ctrl = '{alufnt::ctz,  opr2t::imm};
            {uopc::cbsxt, 5'b??010} : ctrl = '{alufnt::cpop, opr2t::imm};
            {uopc::cbsxt, 5'b??100} : ctrl = '{alufnt::sxtb, opr2t::imm};
            {uopc::cbsxt, 5'b??101} : ctrl = '{alufnt::sxth, opr2t::imm};
            {uopc::pack,  5'b?????} : ctrl = '{alufnt::zxth, opr2t::imm};

            default                 : ctrl = '{alufnt::add,  opr2t::imm};
        endcase
    end

endmodule : alu_decode

module bru_decode (
    input uopc::micro_opcode_t uopcode,
    output brfnt::br_func_t brfn
);

    always_comb begin
        unique case (uopcode)
            uopc::auipc  : brfn = brfnt::none;

            // should be taken care of in decode
            uopc::jal    : brfn = brfnt::jal;
            uopc::jalr   : brfn = brfnt::jal;

            uopc::beq    : brfn = brfnt::beq;
            uopc::bne    : brfn = brfnt::bne;
            uopc::blt    : brfn = brfnt::blt;
            uopc::bge    : brfn = brfnt::bge;
            uopc::bltu   : brfn = brfnt::bltu;
            uopc::bgeu   : brfn = brfnt::bgeu;

            default      : brfn = brfnt::none;
        endcase
    end


endmodule : bru_decode

module alu_imm_dec (
    input   logic [19:0]       packed_imm,
    input   immt::imm_type_t   imm_type,
    output  rv32i_word         imm
);

    logic sign;
    assign sign = packed_imm[19];

    assign imm[31]    = sign;
    assign imm[30:12] = imm_type == immt::u ? packed_imm[18:0] : {19{sign}};
    assign imm[11]    = imm_type == immt::u ? '0 : sign;
    assign imm[10:0]  = imm_type == immt::u ? '0 : packed_imm[18:8];

endmodule : alu_imm_dec

module jmp_imm_dec (
    input   logic [19:0]       packed_imm,
    input   immt::imm_type_t   imm_type,
    output  rv32i_word         imm
);

    logic sign;
    assign sign = packed_imm[19];

    assign imm[31]    = sign;
    assign imm[30:20] = imm_type == immt::u ? packed_imm[18:8] : {11{sign}};
    assign imm[19:12] = imm_type == immt::u || imm_type == immt::j ? packed_imm[7:0] : {8{sign}};
    assign imm[11]    = imm_type == immt::u
                      ? '0
                      : imm_type == immt::j || imm_type == immt::b 
                        ? packed_imm[8] 
                        : sign;
    assign imm[10:5]  = imm_type == immt::u ? '0 : packed_imm[18:13];
    assign imm[4:1]   = imm_type == immt::u ? '0 : packed_imm[12:9];
    assign imm[0]     = 1'b0;

endmodule : jmp_imm_dec

module imm_dec (
    input   logic [19:0]       packed_imm,
    input   immt::imm_type_t   imm_type,
    output  rv32i_word         imm
);

    logic sign;
    assign sign = packed_imm[19];

    assign imm[31]    = sign;
    assign imm[30:20] = imm_type == immt::u ? packed_imm[18:8] : {11{sign}};
    assign imm[19:12] = imm_type == immt::u || imm_type == immt::j ? packed_imm[7:0] : {8{sign}};
    assign imm[11]    = imm_type == immt::u
                      ? '0
                      : imm_type == immt::j || imm_type == immt::b 
                        ? packed_imm[8] 
                        : sign;
    assign imm[10:5]  = imm_type == immt::u ? '0 : packed_imm[18:13];
    assign imm[4:1]   = imm_type == immt::u ? '0 : packed_imm[12:9];
    assign imm[0]     = imm_type == immt::i ? packed_imm[8] : 1'b0;

endmodule : imm_dec
