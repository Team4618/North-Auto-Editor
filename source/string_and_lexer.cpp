struct lexer
{
   char *at;
};

enum token_type
{
   Token_Comma,
   Token_Period,
   Token_Semicolon,
   Token_Equals,
   Token_Asterisk,
   Token_Ampersand,
   Token_Plus,
   Token_Minus,
   Token_GreaterThan,
   Token_LessThan,
   Token_OpenParen,
   Token_CloseParen,
   Token_OpenBrace,
   Token_CloseBrace,
   Token_OpenBracket,
   Token_CloseBracket,
   Token_Number,
   Token_String,
   Token_Identifier,
   Token_NewLine,
   Token_End
};

struct token
{
   char *text;
   u32 length;
   token_type type;
};

void EatAllWhitespace(lexer *lex)
{
   while(IsWhitespace(*lex->at))
      lex->at++;
}

bool TokenIdentifier(token text_token, string identifier_value);

token GetToken(lexer *lex)
{
   EatAllWhitespace(lex);
   token result = {};

   result.text = lex->at;
   result.length = 1;

   switch(*lex->at)
   {
      case ',': {result.type = Token_Comma; lex->at++;} break;
      case '.': {result.type = Token_Period; lex->at++;} break;
      case ';': {result.type = Token_Semicolon; lex->at++;} break;
      case '=': {result.type = Token_Equals; lex->at++;} break;
      case '*': {result.type = Token_Asterisk; lex->at++;} break;
      case '&': {result.type = Token_Ampersand; lex->at++;} break;
      case '+': {result.type = Token_Plus; lex->at++;} break;
      case '-': {result.type = Token_Minus; lex->at++;} break;
      case '>': {result.type = Token_GreaterThan; lex->at++;} break;
      case '<': {result.type = Token_LessThan; lex->at++;} break;
      case '(': {result.type = Token_OpenParen; lex->at++;} break;
      case ')': {result.type = Token_CloseParen; lex->at++;} break;
      case '{': {result.type = Token_OpenBrace; lex->at++;} break;
      case '}': {result.type = Token_CloseBrace; lex->at++;} break;
      case '[': {result.type = Token_OpenBracket; lex->at++;} break;
      case ']': {result.type = Token_CloseBracket; lex->at++;} break;
      case '\n': {result.type = Token_NewLine; lex->at++;} break;
      case '\0': {result.type = Token_End;} break;

      case '"':
      {
         lex->at++;
         result.text = lex->at;
         result.length = 0;
         result.type = Token_String;

         while(lex->at[0] && (lex->at[0] != '"'))
         {
            if((lex->at[0] == '\\') && lex->at[1])
            {
               lex->at++;
               result.length++;
            }

            lex->at++;
            result.length++;
         }

         lex->at++;
      }
      break;

      default:
      {
         if(IsAlpha(*lex->at))
         {
            result.type = Token_Identifier;
            lex->at++;

            while(IsAlpha(*lex->at) || IsNumeric(*lex->at) || (*lex->at == '_'))
            {
               lex->at++;
               result.length++;
            }
         }
         else if(IsNumeric(*lex->at))
         {
            result.type = Token_Number;
            lex->at++;

            while(IsNumeric(*lex->at))
            {
               lex->at++;
               result.length++;
            }
         }
      }
      break;
   }

   return result;
}

token PeekToken(lexer *lex)
{
   lexer temp_lex = *lex;
   return GetToken(&temp_lex);
}

string Literal(token token_in)
{
   string result = {};

   result.text = token_in.text;
   result.length = token_in.length;

   return result;
}

s32 ParseS32(lexer *lex) {
   token first = GetToken(lex);
   bool negative = false;
   u32 num = 0;
   if(first.type == Token_Minus) {
      negative = true;
      num = ToU32(Literal(GetToken(lex)));
   } else {
      num = ToU32(Literal(first));
   }
   
   return (negative ? -1 : 1) * ((s32) num);
}

bool TokenIdentifier(token text_token, string identifier_value)
{
   if(text_token.type != Token_Identifier) return false;
   return Literal(text_token) == identifier_value;
}

void EatTokens(lexer *lex, u32 count)
{
   for(u32 i = 0; i < count; i++)
      GetToken(lex);
}