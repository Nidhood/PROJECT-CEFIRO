#!/usr/bin/env python3
"""No rclpy get_logger call with extra printf-style positional arguments."""
import ast
from pathlib import Path
p=Path(__file__).resolve().parents[1]/'scripts/cefiro_balance_node.py'
t=ast.parse(p.read_text(),filename=str(p))
count=0
for call in ast.walk(t):
    if (isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute)
        and call.func.attr in {'info','debug','warn','warning','error','fatal'}
        and isinstance(call.func.value,ast.Call)
        and isinstance(call.func.value.func,ast.Attribute)
        and call.func.value.func.attr=='get_logger'):
        assert len(call.args)==1,(call.lineno,len(call.args))
        count+=1
assert count>=4
print('PASS: %d rclpy log calls'%count)
