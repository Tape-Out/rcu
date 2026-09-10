package Rcu;

import Vector::*;
import RegIf::*;
import RcuRegs::*;

// 本包不认识任何总线：对外只给中立的 RegIf，接哪种总线由 wrap 或装配决定。
typedef struct {
  Bool pll;
} RcuCfg;

// 门控给的是**使能**不是时钟：真正的门控单元是 PDK 里的 ICG，由装配处例化，
// 这里只出控制线。自己在逻辑里与一个时钟做与，会得到带毛刺的时钟。
interface RcuPins#(numeric type domains);
  (* always_ready, result = "clk_en"  *) method Bit#(domains) clk_en;
  (* always_ready, result = "rst_n"   *) method Bit#(domains) rst_n;
  (* always_ready, result = "pll_en"  *) method Bit#(1) pll_en;
  (* always_ready, result = "pll_m"   *) method Bit#(7) pll_m;
  (* always_ready, result = "pll_r"   *) method Bit#(6) pll_r;
  (* always_ready, always_enabled, prefix = "" *)
  method Action pll_locked((* port = "pll_locked" *) Bit#(1) v);
endinterface

interface RcuIfc#(numeric type aw, numeric type dw, numeric type domains);
  interface RegIf#(aw, dw) regs;
  interface RcuPins#(domains) pins;
endinterface

module mkRcu#(RcuCfg cfg)(RcuIfc#(aw, dw, domains))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, domains, dw),
              Add#(_c, 8, dw), Add#(_d, 7, dw), Add#(_e, 6, dw), Add#(_f, 1, dw));

  RcuRegsIfc#(aw, dw, domains) r <- mkRcuRegs(RcuRegsCfg { pll: cfg.pll });

  // 每个域一个分频计数器，计到 div 就放一拍使能
  Vector#(domains, Reg#(Bit#(8))) cnt <- replicateM(mkReg(0));
  Reg#(Bit#(domains)) beat <- mkReg(0);

  rule divide;
    Bit#(domains) b = 0;
    for (Integer i = 0; i < valueOf(domains); i = i + 1) begin
      Bit#(8) top = (r.div[i] == 0) ? 0 : r.div[i] - 1;
      if (cnt[i] >= top) begin
        cnt[i] <= 0;
        b[i] = 1;
      end else
        cnt[i] <= cnt[i] + 1;
    end
    beat <= b;
  endrule

  Wire#(Bit#(1)) locked <- mkBypassWire;

  if (cfg.pll) begin
    rule lockStatus;
      r.pllcfg_lock_in(locked);
    endrule
  end

  // 复位要等时钟稳。PLL 没锁定就放开，逻辑会在时钟还没稳的时候开始跑——
  // 「复位时序」这四个字指的就是这件事，而 pll_locked 此前只被拿去填状态位。
  // 倍频为一时不过 PLL，不必等；特性关掉时这一支整个不例化。
  function Bit#(domains) rstOut();
    Bit#(domains) o = r.rstn;
    if (cfg.pll && r.pllcfg_mult != 1 && locked == 0) o = 0;
    return o;
  endfunction

  interface regs = r.regs;
  interface RcuPins pins;
    method Bit#(domains) clk_en = r.gate & beat;
    method Bit#(domains) rst_n  = rstOut();
    // 配了非一的倍频就当作要用 PLL
    method Bit#(1) pll_en = (cfg.pll && r.pllcfg_mult != 1) ? 1 : 0;
    method Bit#(7) pll_m  = cfg.pll ? r.pllcfg_mult : 1;
    method Bit#(6) pll_r  = cfg.pll ? r.pllcfg_divr : 1;
    method Action pll_locked(Bit#(1) v); locked._write(v); endmethod
  endinterface
endmodule

endpackage
