#include "all.h"

/*
	调试日志缓冲实现 (从机侧)
	==================================================================
	所有函数都只是往 RAM 里搬字符, 不碰串口。
	只有 LOG_Flush() 会真正阻塞发送 —— 那是刻意的, 它被放在主循环最后一步。
*/

static char s_buf[LOG_BUF_SIZE];
static u16  s_len     = 0;      /* 已攒字节数 */
static u16  s_dropped = 0;      /* 因缓冲满被丢弃的字节数 */

/* 给截断标记留的尾部空间, 保证标记一定写得进去 */
#define LOG_TAIL_RESERVE (32)

void LOG_Init(void)
{
	s_len     = 0;
	s_dropped = 0;
	s_buf[0]  = 0;
}

/*
	写入一个字符
	★ 缓冲留 LOG_TAIL_RESERVE 字节余量给"截断标记",
	  这样即使日志写爆了, 结尾也能完整打出 [TRUNC], 不会出现半截字符串。
*/
static void log_putc(char c)
{
	if(s_len < (u16)(LOG_BUF_SIZE - LOG_TAIL_RESERVE))
	{
		s_buf[s_len++] = c;
	}
	else
	{
		s_dropped += 1;
	}
}

void LOG_Str(const char *s)
{
	if(s == 0) return;
	while(*s)
	{
		log_putc(*s);
		s += 1;
	}
}

void LOG_Chr(char c)
{
	log_putc(c);
}

void LOG_Num(u16 n)
{
	char b[6];
	u8 i = 0;

	if(n == 0)
	{
		log_putc('0');
		return;
	}
	while(n > 0)
	{
		b[i++] = (char)('0' + (n % 10));
		n /= 10;
	}
	while(i > 0)
	{
		log_putc(b[--i]);
	}
}

void LOG_HexNum(u16 v)
{
	const char *hx = "0123456789ABCDEF";

	log_putc(hx[(v >> 12) & 0x0F]);
	log_putc(hx[(v >>  8) & 0x0F]);
	log_putc(hx[(v >>  4) & 0x0F]);
	log_putc(hx[ v        & 0x0F]);
}

/*
	十六进制转储
	-----------------------------------------------------------------
	★ v20 修: 长度字段原来是手写的两位十进制, len 被截到 48 时没问题,
	  但那个 "if(len >= 10)" 只处理十位, 一旦以后把上限改到 >99 就会
	  打出乱码字符(比如 len=100 时 '0'+10 = ':' )。现在直接用 LOG_Num,
	  任意长度都正确。
	★ 另一个改动: 上限从 48 提到 64, 因为从机最长应答 17 字节、
	  主机最长请求/应答 17 字节都够用, 但带 CRC 的完整上报帧会更长,
	  64 给以后留余量。同时仍然限制上限, 防止一条日志吃掉整个缓冲。
*/
void LOG_HexDump(const char *tag, u8 *buf, u16 len)
{
	const char *hx = "0123456789ABCDEF";
	u16 i = 0;
	u8 hi = 0;
	u16 shown = 0;

	if(tag == 0 || buf == 0) return;

	shown = (len > 64) ? 64 : len;   /* 防止一条日志吃掉整个缓冲 */

	LOG_Str(tag);
	LOG_Chr('[');
	/*
		★ 长度格式化: <100 时补成两位(8 -> "08"), >=100 时原样。
		  为什么要补零: 日志是给人快速扫的, "RX[08]" 和 "TX[17]" 竖着看
		  是对齐的; 混着 "RX[8]" 和 "TX[17]" 眼睛会跳。
		  为什么不用 "%02d" 那种写法: 这里没有 printf, 只能手写,
		  而手写两位数时 len=100 会打出 '0'+10 = ':' 这种乱码 ——
		  所以必须分成两段判断, 不能只判 >=10。
	*/
	if(shown < 100)
	{
		LOG_Chr((char)('0' + (shown / 10) % 10));
		LOG_Chr((char)('0' + shown % 10));
	}
	else
	{
		LOG_Num(shown);          /* 三位及以上, 直接打十进制 */
	}
	LOG_Chr(']');
	LOG_Chr(':');
	LOG_Chr(' ');

	for(i = 0 ; i < shown ; i++)
	{
		hi = buf[i];
		LOG_Chr(hx[(hi >> 4) & 0x0F]);
		LOG_Chr(hx[hi & 0x0F]);
		LOG_Chr(' ');
	}

	/* 被截断时明确标出来, 免得看日志的人以为整帧就这么长 */
	if(len > shown)
	{
		LOG_Str("..(cut ");
		LOG_Num((u16)(len - shown));
		LOG_Str("B)");
	}

	LOG_Str("\r\n");
}

u8 LOG_Pending(void)
{
	return (u8)((s_len > 0 || s_dropped > 0) ? 1 : 0);
}

/*
	把十进制数字直接写进缓冲的指定位置(绕过 log_putc)
	★ 为什么需要这个: 截断标记必须能写进"预留的尾部空间",
	  而 log_putc 的判据是 s_len < LOG_BUF_SIZE - LOG_TAIL_RESERVE,
	  在缓冲已满时它会把标记本身也丢掉 —— 那样就永远看不到 [TRUNC] 了。
	  (这个 bug 是测试跑出来的: 写爆 200 行后缓冲区里一个 TRUNC 都没有)
*/
static u16 log_put_num_at(u16 pos, u16 n)
{
	char b[6];
	u8 i = 0;

	if(n == 0)
	{
		s_buf[pos++] = '0';
		return pos;
	}
	while(n > 0)
	{
		b[i++] = (char)('0' + (n % 10));
		n /= 10;
	}
	while(i > 0)
	{
		s_buf[pos++] = b[--i];
	}
	return pos;
}

void LOG_Flush(void)
{
	/* 缓冲空的快路径: 绝大多数循环都没日志可发, 不能在这里浪费任何时间 */
	if(s_len == 0 && s_dropped == 0)
	{
		return;
	}

	/*
		有内容被丢弃时, 在结尾标注, 免得看日志的人以为"就这么多"。
		★ 这里【不能】调 LOG_Str/LOG_Num: 缓冲已经满了,
		  它们会走 log_putc 的丢弃分支, 标记自己就被丢掉。
		  所以直接写进预留的尾部空间(LOG_TAIL_RESERVE 字节, 足够放下
		  " ...[TRUNC 65535B]\r\n" 共 20 字节)。
	*/
	if(s_dropped > 0)
	{
		u16 pos = s_len;

		s_buf[pos++] = ' ';
		s_buf[pos++] = '.';
		s_buf[pos++] = '.';
		s_buf[pos++] = '.';
		s_buf[pos++] = '[';
		s_buf[pos++] = 'T';
		s_buf[pos++] = 'R';
		s_buf[pos++] = 'U';
		s_buf[pos++] = 'N';
		s_buf[pos++] = 'C';
		s_buf[pos++] = ' ';
		pos = log_put_num_at(pos, s_dropped);
		s_buf[pos++] = 'B';
		s_buf[pos++] = ']';
		s_buf[pos++] = '\r';
		s_buf[pos++] = '\n';
		s_len = pos;
	}

	s_buf[s_len] = 0;               /* 补字符串结尾 */
	USART1_SendString(s_buf);       /* 唯一的阻塞点 */

	s_len     = 0;
	s_dropped = 0;
}
