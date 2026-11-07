# Submodo publicado pelo supervisor dentro do modo do orquestrador (D-01).
string NONE=NONE
string EXPLORING=EXPLORING
string COVERING=COVERING
string GOTO_ROOM=GOTO_ROOM
string ESCAPING=ESCAPING
string ESCAPE_FAILED=ESCAPE_FAILED

builtin_interfaces/Time stamp
string mode        # cópia do último /myRobot/behaviour/mode recebido
string submode     # uma das constantes acima
string reason      # motivo da última transição, para log e interface
