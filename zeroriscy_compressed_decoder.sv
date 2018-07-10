
import zeroriscy_defines::*;

module zeroriscy_compressed_decoder
(
  input  logic [31:0] instr_i,
  output logic [31:0] instr_o,
  output logic        is_compressed_o,
  output logic        illegal_instr_o
);

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //   ____                                                 _   ____                     _            //
  //  / ___|___  _ __ ___  _ __  _ __ ___  ___ ___  ___  __| | |  _ \  ___  ___ ___   __| | ___ _ __  //
  // | |   / _ \| '_ ` _ \| '_ \| '__/ _ \/ __/ __|/ _ \/ _` | | | | |/ _ \/ __/ _ \ / _` |/ _ \ '__| //
  // | |__| (_) | | | | | | |_) | | |  __/\__ \__ \  __/ (_| | | |_| |  __/ (_| (_) | (_| |  __/ |    //
  //  \____\___/|_| |_| |_| .__/|_|  \___||___/___/\___|\__,_| |____/ \___|\___\___/ \__,_|\___|_|    //
  //                      |_|                                                                         //
  //////////////////////////////////////////////////////////////////////////////////////////////////////

  always_comb
  begin
    illegal_instr_o = 1'b0;
    instr_o         = '0;

    unique case (instr_i[1:0])
      // C0
      2'b00: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.addi4spn -> addi rd', x2, imm
            instr_o = {3'b0, 1'b0, 4'h2, {2'b0, instr_i[10:7], instr_i[12:11], instr_i[5], instr_i[6], 2'b00}, 3'b000, {1'b1, instr_i[4:2]}, 3'b000, OP_ALU_IMM};
            if (instr_i[12:5] == 8'b0)  illegal_instr_o = 1'b1;
          end

          3'b010: begin
            // c.lw -> lw rd', imm(rs1')
            instr_o = {4'b0, {1'b1, instr_i[9:7]}, {5'b0, instr_i[5], instr_i[12:10], instr_i[6], 2'b00}, {1'b1, instr_i[4:2]}, 3'b010, OP_LOAD};
          end

          3'b110: begin
            // c.sw -> sw rs2', imm(rs1')
            instr_o = {4'b0, {1'b1, instr_i[9:7]}, {1'b0, instr_i[4:2]}, {1'b0, instr_i[5], instr_i[12:10], instr_i[6], 2'b00}, 4'b0, 3'b010, OP_STORE};
          end

          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      // C1
      2'b01: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.addi -> addi rd, rd, nzimm
            // c.nop
            instr_o = {3'b0, 1'b0, instr_i[10:7], {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2]}, instr_i[10:7], 3'b000, OP_ALU_IMM};
            if (instr_i[11] != 0) illegal_instr_o = 1'b1;
          end

          3'b001, 3'b101: begin
            // 001: c.jal -> jal x1, imm
            // 101: c.j   -> jal x0, imm
            instr_o = {3'b0, ~instr_i[15], {{4 {instr_i[12]}}, instr_i[12], instr_i[8], instr_i[10:9], instr_i[6], instr_i[7], instr_i[2], instr_i[11], instr_i[5:3], instr_i[12]}, {4 {instr_i[12]}}, OPCODE_JAL, OP_U_IMM};
          end

          3'b010: begin
            // c.li -> addi rd, x0, nzimm
            instr_o = {3'b0, 1'b0, 4'b0, {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2]}, instr_i[10:7], 3'b000, OP_ALU_IMM};
            if (instr_i[11:7] == 5'b0 || instr_i[11] != 0)  illegal_instr_o = 1'b1;
          end

          3'b011: begin
            // c.lui -> lui rd, imm
            instr_o = {{{15 {instr_i[12]}}, instr_i[6:2]}, instr_i[10:7], OPCODE_LUI, OP_U_IMM};

            if (instr_i[11:7] == 5'h02) begin
              // c.addi16sp -> addi x2, x2, nzimm
              instr_o = {3'b0, 1'b0, 4'h2, {{3 {instr_i[12]}}, instr_i[4:3], instr_i[5], instr_i[2], instr_i[6]}, 4'h2, 3'b0, OP_ALU_IMM};
            end else if (instr_i[11:7] == 5'b0) begin
              illegal_instr_o = 1'b1;
            end

            if ({instr_i[12], instr_i[6:2]} == 6'b0) illegal_instr_o = 1'b1;
            if (instr_i[11] != 0) illegal_instr_o = 1'b1;
          end

          3'b100: begin
            unique case (instr_i[11:10])
              2'b00,
              2'b01: begin
                // 00: c.srli -> srli rd, rd, shamt
                // 01: c.srai -> srai rd, rd, shamt
                instr_o = {3'b0, 1'b0,{1'b1, instr_i[9:7]}, 3'b0, instr_i[6:2], instr_i[10], 3'b101, {1'b1, instr_i[9:7]}, OPCODE_SHIFT, OP_ALU_REG};
                if (instr_i[12] == 1'b1)  illegal_instr_o = 1'b1;
                if (instr_i[6:2] == 5'b0) illegal_instr_o = 1'b1;
              end

              2'b10: begin
                // c.andi -> andi rd, rd, imm
                instr_o = {3'b0, 1'b0, {1'b1, instr_i[9:7]}, {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2]}, {1'b1, instr_i[9:7]}, 3'b0, OP_ALU_IMM};
              end

              2'b11: begin
                unique case ({instr_i[12], instr_i[6:5]})
                  3'b000: begin
                    // c.sub -> sub rd', rd', rs2'
                    instr_o = {3'b0, 1'b0, {1'b1, instr_i[9:7]},{1'b1, instr_i[4:2]}, 4'b0, 1'b1, 3'b000, {1'b1, instr_i[9:7]}, 3'b0, OP_ALU_REG};
                  end

                  3'b001: begin
                    // c.xor -> xor rd', rd', rs2'
                    instr_o = {3'b0, 1'b0, {1'b1, instr_i[9:7]},{1'b1, instr_i[4:2]}, 4'b0, 1'b0, 3'b100, {1'b1, instr_i[9:7]}, 3'b0, OP_ALU_REG};
                  end

                  3'b010: begin
                    // c.or  -> or  rd', rd', rs2'
                    instr_o = {3'b0, 1'b0, {1'b1, instr_i[9:7]},{1'b1, instr_i[4:2]}, 4'b0, 1'b0, 3'b110, {1'b1, instr_i[9:7]}, 3'b0, OP_ALU_REG};
                  end

                  3'b011: begin
                    // c.and -> and rd', rd', rs2'
                    instr_o = {3'b0, 1'b0, {1'b1, instr_i[9:7]},{1'b1, instr_i[4:2]}, 4'b0, 1'b1, 3'b111, {1'b1, instr_i[9:7]}, 3'b0, OP_ALU_REG};
                  end

                  3'b100,
                  3'b101,
                  3'b110,
                  3'b111: begin
                    // 100: c.subw
                    // 101: c.addw
                    illegal_instr_o = 1'b1;
                  end
                endcase
              end
            endcase
          end

          3'b110, 3'b111: begin
            // 0: c.beqz -> beq rs1', x0, imm
            // 1: c.bnez -> bne rs1', x0, imm
            instr_o = {3'b0, 1'b0, {1'b1, instr_i[9:7]}, 4'b0, {instr_i[6:5], instr_i[2], instr_i[11:10], instr_i[4:3], instr_i[12]}, {4 {instr_i[12]}},{2'b00, instr_i[13]}, OP_BRANCH};
          end

          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      // C2
      2'b10: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.slli -> slli rd, rd, shamt
            instr_o = {3'b0, 1'b0, instr_i[10:7], 3'b0, instr_i[6:2], 1'b0, 3'b001, {1'b1, instr_i[9:7]}, OPCODE_SHIFT, OP_ALU_REG};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
            if (instr_i[12] == 1'b1 || instr_i[6:2] == 5'b0)  illegal_instr_o = 1'b1;
            if (instr_i[11] != 0) illegal_instr_o = 1'b1;
          end

          3'b010: begin
            // c.lwsp -> lw rd, imm(x2)
            instr_o = {4'b0, 4'h2, {4'b0, instr_i[3:2], instr_i[12], instr_i[6:4], 2'b00}, instr_i[10:7], 3'b010, OP_LOAD};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b100: begin
            if (instr_i[12] == 1'b0) begin
              // c.mv -> add rd/rs1, x0, rs2
              instr_o = {3'b0, 1'b0, 4'b0, instr_i[5:2], 4'b0, 1'b0, 3'b0, instr_i[10:7], 3'b000, OP_ALU_REG};
              
              if (instr_i[6:2] == 5'b0) begin
                // c.jr -> jalr x0, rd/rs1, 0
                instr_o = {3'b0, 1'b0, 4'b0, 12'b0, instr_i[10:7], OPCODE_JALR, OP_BRANCH};
              end       
            end else begin
              // c.add -> add rd, rd, rs2
              instr_o = {3'b0, 1'b0, instr_i[10:7], instr_i[5:2], 4'b0, 1'b0, 3'b0, instr_i[10:7], 3'b0, OP_ALU_REG};

              if (instr_i[11:7] == 5'b0) begin
                // c.ebreak -> ebreak
                instr_o = {32'h00_001_0_fb};
                if (instr_i[6:2] != 5'b0)
                  illegal_instr_o = 1'b1;
              end else if (instr_i[6:2] == 5'b0) begin
                // c.jalr -> jalr x1, rs1, 0
                instr_o = {3'b0, 1'b0, instr_i[10:7], 3'b000, 12'b0, 4'b0001, OPCODE_JALR, OP_BRANCH};
              end
            end
            if (instr_i[6] != 0 || instr_i[11] != 0) illegal_instr_o = 1'b1;     
          end

          3'b110: begin
            // c.swsp -> sw rs2, imm(x2)
            instr_o = {4'b0, 4'h2, instr_i[5:2], {instr_i[8:7], instr_i[12], instr_i[11:9], 2'b0}, 4'b0, 3'b010, OP_STORE};
            if (instr_i[6] != 0) illegal_instr_o = 1'b1;   
          end

          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      default: begin
        // 32 bit (or more) instruction
        instr_o = instr_i;
      end
    endcase
  end

  assign is_compressed_o = (instr_i[1:0] != 2'b11);

endmodule
