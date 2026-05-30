# Emitter byte-equivalence verification

Gate for the emitter cursor-passing refactor (see
`emitter-cursor-refactor-scope.md`). Emits a corpus of instruction shapes
through the real emitter and dumps the bytes. Any change to the emitter that
is meant to be behaviour-preserving MUST produce output identical to
`golden.txt`.

## Run
```
# from repo root
for f in common/emitter/*.cpp; do
  g++ -std=c++17 -O2 -c -I common -I common/emitter -I. "$f" -o /tmp/$(basename $f .cpp).o
done
g++ -std=c++17 -O2 -I common -I common/emitter -I. \
    tools/emitter-verify/emit_corpus.cpp /tmp/*.o -o /tmp/emit_corpus
/tmp/emit_corpus > /tmp/after.txt
diff tools/emitter-verify/golden.txt /tmp/after.txt && echo "PASS: byte-identical"
```

A non-empty diff is a refactor bug. Regenerate `golden.txt` only when the
corpus itself is intentionally extended (never to paper over an encoding
change).

## Coverage
ALU reg-reg / reg-imm (imm8 + imm32) across 8/16/32/64-bit and extended
registers (REX.R/B/X); every memory addressing form (base, base+disp8/32,
base+index, scaled index, disp32); SSE reg-reg / reg-mem with 0x66/0x0f
prefixes and extended XMM. Extend as needed for shapes the refactor touches.
