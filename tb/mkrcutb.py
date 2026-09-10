"""rcu 的行为测试台：分频比真的按域各自算、门控关得掉、复位线直通、PLL 那组。

分频用数拍来验：一号域除一，每拍都该放行；二号域除四，四拍里只该放行一次。
比「看某一拍高不高」稳，不跟采样相位较劲。

门控给的是**使能**不是时钟——真正的门控单元是 PDK 里的 ICG，装配处例化。
所以这里验的是使能线，不是波形。

认矩阵：`domains` 与 `pll` 从这一点的旋钮来。pll 关掉时期望反过来：
配置寄存器读回零、pll_en 恒零、倍频与分频回到 1。
"""
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
k = cfg.get("knobs", {})
doms = int(k.get("domains", 4))
pll = bool(k.get("pll", False))

N = 40           # 数多少拍
second = doms >= 2
# 复位放开的图案要放得进 domains 位：三个域以上才用得上 0 号与 2 号
RSTV = 5 if doms >= 3 else 1

div1_setup = ("      2: wr(8'h14, 4);            // div[1] 除四"
              if second else "      2: noAction;")
beat1 = ("      if (e[1] == 1) n1[0] <= n1[0] + 1;" if second
         else "      // 只有一个域")

# PLL 没锁定就放开复位，逻辑会在时钟还没稳的时候开始跑。这一段只在 pll 开着时有意义。
lock_phases = ("Unlock, CheckUnlock, Relock, CheckRelock, " if pll else "")
lock_next = "Unlock" if pll else "Done"
lock_rules = ('''
  // PLL 还没锁定就放开复位，逻辑会在时钟不稳的时候开始跑。这一路此前从没被走过——
  // 测试台把 pll_locked 焊死成 1，而实现只把它拿去填状态位。
  rule unlock (ph == Unlock);
    if (s == 0) lockIn <= 0;
    if (s > 6) begin ph <= CheckUnlock; s <= 0; end
    else s <= s + 1;
  endrule

  rule checkUnlock (ph == CheckUnlock);
    if (rstSeen[1] != 0) begin
      $display("FAIL reset released while the pll was unlocked: rst_n is %08h",
               rstSeen[1]);
      bad <= True;
    end
    ph <= Relock;
    s  <= 0;
  endrule

  rule relock (ph == Relock);
    if (s == 0) lockIn <= 1;
    if (s > 6) begin ph <= CheckRelock; s <= 0; end
    else s <= s + 1;
  endrule

  // 反过来也要验：锁上之后复位必须真的放开，否则「一直压着」也能骗过上一条
  rule checkRelock (ph == CheckRelock);
    if (rstSeen[1] == 0) begin
      $display("FAIL the pll locked but reset stayed asserted");
      bad <= True;
    end
    ph <= Done;
  endrule
''' if pll else "")

div1_check = ('''    // 二号域除四：四拍里只该放行一次，宽松点判也足够分得开
    if (n1[1] > (n0[1] >> 1) || n1[1] < 4) begin
      $display("FAIL domain 1 divides by four but beat %0d of %0d cycles",
               n1[1], n0[1]);
      wrong = True;
    end''' if second else "    // 只有一个域，没有第二个分频可比")

arr_check = ('''    if (x.rdata[7:0] != 1) begin
      $display("FAIL div[0] is %02h, want 01 (the array aliases)", x.rdata[7:0]);
      wrong = True;
    end''' if second else "    // 只有一个域，没有数组可对")

if pll:
    pll_check = '''    if (penSeen[1] != 1) begin
      $display("FAIL a multiplier was configured but pll_en stayed low");
      wrong = True;
    end
    if (pmSeen[1] != 8 || prSeen[1] != 2) begin
      $display("FAIL pll m/r are %0d/%0d, want 8/2", pmSeen[1], prSeen[1]);
      wrong = True;
    end'''
    verdict = "each domain divides on its own, the gate gates, and the pll lines follow"
else:
    pll_check = '''    if (penSeen[1] != 0) begin
      $display("FAIL the pll is off but pll_en went high");
      wrong = True;
    end
    if (pmSeen[1] != 1 || prSeen[1] != 1) begin
      $display("FAIL the pll is off but m/r are %0d/%0d, want 1/1",
               pmSeen[1], prSeen[1]);
      wrong = True;
    end'''
    verdict = "each domain divides on its own, the gate gates, and the pll gate really gates"

txt = f'''package Rcu{label}Tb;

import RegIf::*;
import Rcu::*;

// 由 tb/mkrcutb.py 生成，勿手改。这一点：domains={doms} pll={pll}

typedef enum {{ Setup, Count, Gate, GateCheck, Check, {lock_phases}Done }}
  Phase deriving (Bits, Eq);

(* synthesize *)
module mkRcu{label}Tb(Empty);
  RcuIfc#(8, 32, {doms}) d <- mkRcu(RcuCfg {{ pll: {"True" if pll else "False"} }});

  Reg#(Phase)    ph  <- mkReg(Setup);
  Reg#(Bit#(16)) s   <- mkReg(0);
  Reg#(Bit#(32)) cyc <- mkReg(0);
  Reg#(Bool)     bad <- mkReg(False);
  // 采样规则每拍都跑，凡是它写、检查规则读的量都得用 CReg
  Reg#(Bit#(16)) n0[2] <- mkCReg(2, 0);
  Reg#(Bit#(16)) n1[2] <- mkCReg(2, 0);
  Reg#(Bit#({doms})) rstSeen[2] <- mkCReg(2, 0);
  Reg#(Bit#({doms})) enSeen[2]  <- mkCReg(2, 0);
  Reg#(Bit#(1)) penSeen[2] <- mkCReg(2, 0);
  Reg#(Bit#(7)) pmSeen[2]  <- mkCReg(2, 0);
  Reg#(Bit#(6)) prSeen[2]  <- mkCReg(2, 0);

  // 锁定信号由测试台驱动，好把「还没锁定」那一路走一遍
  Reg#(Bit#(1)) lockIn <- mkReg(1);

  rule drivePins;
    d.pins.pll_locked(lockIn);
  endrule

  rule samplePins;
    Bit#({doms}) e = d.pins.clk_en;
    rstSeen[0] <= d.pins.rst_n;
    enSeen[0]  <= e;
    penSeen[0] <= d.pins.pll_en;
    pmSeen[0]  <= d.pins.pll_m;
    prSeen[0]  <= d.pins.pll_r;
    if (ph == Count) begin
      if (e[0] == 1) n0[0] <= n0[0] + 1;
{beat1}
    end
  endrule

  rule tick_;
    cyc <= cyc + 1;
    if (cyc > 40000) begin
      $display("TIMEOUT in phase %0d", pack(ph));
      $finish(1);
    end
  endrule

  function Action wr(Bit#(8) a, Bit#(32) v) = action
    let _ <- d.regs.access(RegReq {{ addr: a, write: True,
                                     wdata: v, wstrb: 4'hF }});
  endaction;

  rule setup (ph == Setup);
    case (s)
      0: wr(8'h00, 32'hFFFFFFFF);   // gate 全开
      1: wr(8'h10, 1);              // div[0] 除一
{div1_setup}
      3: wr(8'h04, {RSTV});         // rstn 放开的图案
      4: wr(8'h60, 32'h00000208);   // pllcfg：mult = 8，divr = 2
      default: ph <= Count;
    endcase
    if (s < 5) s <= s + 1; else s <= 0;
  endrule

  rule count (ph == Count);
    if (s > {N}) begin ph <= Gate; s <= 0; end
    else s <= s + 1;
  endrule

  rule gate (ph == Gate);
    wr(8'h00, 0);                   // gate 全关
    ph <= GateCheck;
    s  <= 0;
  endrule

  rule gateCheck (ph == GateCheck);
    if (s > 4) begin
      if (enSeen[1] != 0) begin
        $display("FAIL the gate is closed but clk_en is %08h", enSeen[1]);
        bad <= True;
      end
      ph <= Check;
      s  <= 0;
    end else s <= s + 1;
  endrule

  rule check (ph == Check);
    let x <- d.regs.access(RegReq {{ addr: 8'h10, write: False,
                                     wdata: 0, wstrb: 4'hF }});
    Bool wrong = False;
    // 一号域除一：每一拍都该放行
    if (n0[1] < {N - 2}) begin
      $display("FAIL domain 0 divides by one but beat only %0d of {N} cycles",
               n0[1]);
      wrong = True;
    end
{div1_check}
    if (rstSeen[1] != {doms}'h{RSTV:X}) begin
      $display("FAIL rst_n is %08h, want %02h", rstSeen[1], {doms}'h{RSTV:X});
      wrong = True;
    end
{pll_check}
{arr_check}
    if (wrong) bad <= True;
    ph <= {lock_next};
    s  <= 0;
  endrule
{lock_rules}
  rule fin (ph == Done);
    if (bad) $display("FAILED");
    else $display("PASS rcu: {verdict}");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

(out / f"Rcu{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  rcu 行为测试台就位：domains={doms} pll={pll}")
